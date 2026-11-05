// =============================================================================
// Контроллер EEPROM AT24C08 поверх i2c_master: запись с ACK-polling и чтение
//
// Запрос (valid/ready), принимается в такте req_valid = 1 и req_ready = 1:
//   req_op   - операция:
//                0  WR_BYTE   записать байт req_data[7:0] по адресу req_addr;
//                1  WR_WORD   записать слово req_data[15:0] по адресам req_addr и
//                             req_addr + 1: старший байт по меньшему адресу (big-endian,
//                             как в PROTOCOL.md). Если слово пересекает границу страницы
//                             (первый байт - последний в странице), оно записывается
//                             двумя отдельными циклами записи;
//                2  RD_BYTE   прочитать один байт по адресу req_addr (произвольное чтение);
//                3  RD_SEQ    прочитать req_len байт подряд, начиная с адреса req_addr
//                             (произвольное чтение первого байта, дальше микросхема сама
//                             увеличивает адрес). Внутри одного блока в 256 байт всегда
//                             работает; переход через границу блока зависит от микросхемы;
//                4  RD_CUR    прочитать req_len байт с текущего адреса микросхемы (без
//                             записи адреса); из req_addr используются только старшие биты
//                             (номер блока в адресе устройства);
//   req_addr - адрес ячейки, ADDR_W бит (AT24C08: 10 бит, старшие 2 бита - номер блока
//              в адресе устройства DEV_ADDR + блок);
//   req_len  - число байт для RD_SEQ и RD_CUR (0 означает 1).
// После записи контроллер ждёт конец внутреннего цикла записи микросхемы: посылает
// адрес устройства (запись 0 байт) и повторяет, пока микросхема отвечает NACK, но не
// больше POLL_MAX раз.
//
// Поток чтения (valid/ready): data - прочитанный байт, data_last = 1 на последнем байте
//   запроса. Пока data_ready = 0, SCL удерживается в 0.
// Итог - импульс op_done; флаги держатся до следующего запроса:
//   op_ok    - операция выполнена;
//   op_nack  - микросхема не ответила на адрес или байт (нет на шине, не принимает запись);
//   op_bus   - тайм-аут растяжки SCL или шина занята (timeout, arb в i2c_master);
//   op_poll  - после записи микросхема не освободилась за POLL_MAX опросов;
//   op_polls - сколько опросов закончилось NACK (микросхема была занята) за операцию;
//   polling  - уровень: идёт ожидание конца цикла записи.
//
// Интерфейс к i2c_master повторяет порты i2c_master (команда, поток записи, поток
// чтения, итог транзакции), как у dt_reader.
// =============================================================================
module eeprom_ctrl #(
    parameter int         LEN_W      = 8,
    parameter int         ADDR_W     = 10,           // разрядность адреса ячейки (не меньше 9)
    parameter logic [6:0] DEV_ADDR   = 7'h50,        // адрес устройства для блока 0
    parameter int         PAGE_BYTES = 16,           // размер страницы записи, степень двойки
    parameter int         POLL_MAX   = 100           // предел опросов ACK после записи (не больше 255)
) (
    input  logic              clk,
    input  logic              rst_n,         // асинхронный сброс, активный 0

    // запрос
    input  logic              req_valid,
    output logic              req_ready,
    input  logic [2:0]        req_op,
    input  logic [ADDR_W-1:0] req_addr,
    input  logic [15:0]       req_data,
    input  logic [LEN_W-1:0]  req_len,

    // поток чтения
    output logic              data_valid,
    input  logic              data_ready,
    output logic [7:0]        data,
    output logic              data_last,

    // итог операции
    output logic              op_done,
    output logic              op_ok,
    output logic              op_nack,
    output logic              op_bus,
    output logic              op_poll,
    output logic [7:0]        op_polls,
    output logic              polling,

    // команды и поток записи для i2c_master
    output logic              cmd_valid,
    input  logic              cmd_ready,
    output logic [6:0]        cmd_addr,
    output logic              cmd_read,
    output logic [LEN_W-1:0]  cmd_len,
    output logic              cmd_stop,
    output logic [7:0]        wr_data,
    output logic              wr_valid,
    input  logic              wr_ready,

    // поток чтения от i2c_master
    input  logic [7:0]        rd_data,
    input  logic              rd_valid,
    output logic              rd_ready,
    input  logic              rd_last,

    // итог транзакции
    input  logic              done,
    input  logic              res_nack_addr,
    input  logic              res_nack_data,
    input  logic              res_timeout,
    input  logic              res_arb
);

    localparam logic [2:0] OP_WR_BYTE = 3'd0,
                           OP_WR_WORD = 3'd1,
                           OP_RD_BYTE = 3'd2,
                           OP_RD_SEQ  = 3'd3,
                           OP_RD_CUR  = 3'd4;

    localparam int PAGE_W = $clog2(PAGE_BYTES);

    // каждая транзакция - два состояния: SEND (выдаём команду) и RUN (данные, ждём итог)
    typedef enum logic [3:0] {
        IDLE,
        WR_SEND,   WR_RUN,      // запись: адрес ячейки и данные, STOP
        POLL_SEND, POLL_RUN,    // ожидание конца цикла записи: адрес устройства, 0 байт
        PTR_SEND,  PTR_RUN,     // чтение: адрес ячейки без STOP
        RD_SEND,   RD_RUN,      // чтение: повторный START и байты данных
        FINISH
    } state_t;

    typedef enum logic [2:0] {
        R_NONE,
        R_OK,
        R_NACK,
        R_BUS,
        R_POLL
    } result_t;

    state_t              state;
    result_t             result;
    logic [2:0]          op_r;
    logic [ADDR_W-1:0]   addr_r;        // адрес ячейки; при записи - адрес текущего куска
    logic [15:0]         data_r;
    logic [LEN_W-1:0]    len_r;
    logic [1:0]          left;          // сколько байт записи ещё не записано
    logic [1:0]          wr_cnt;        // сколько байт отдано мастеру в этой транзакции записи
    logic                pos;           // номер байта данных во всей записи (0 - первый)
    logic [7:0]          polls;

    wire is_word = (op_r == OP_WR_WORD);

    // кусок записи: 2 байта, если слово целиком помещается в странице, иначе по одному
    wire [1:0] chunk_n = (left == 2'd2 && !(&addr_r[PAGE_W-1:0])) ? 2'd2 : 2'd1;

    wire fail_bus  = res_timeout || res_arb;
    wire fail_nack = res_nack_addr || res_nack_data;

    wire [6:0] dev_addr = DEV_ADDR + 7'(addr_r[ADDR_W-1:8]);

    assign req_ready = (state == IDLE);
    assign polling   = (state == POLL_SEND) || (state == POLL_RUN);

    assign cmd_valid = (state == WR_SEND) || (state == POLL_SEND) || (state == PTR_SEND) || (state == RD_SEND);
    assign cmd_addr  = dev_addr;
    assign cmd_read  = (state == RD_SEND);
    assign cmd_stop  = (state != PTR_SEND);
    always_comb begin
        case (state)
            WR_SEND: cmd_len = LEN_W'(chunk_n) + 1'b1;         // адрес ячейки и данные
            PTR_SEND: cmd_len = LEN_W'(1);
            RD_SEND: cmd_len = len_r;
            default: cmd_len = '0;                              // опрос: запись 0 байт
        endcase
    end

    assign wr_valid = (state == WR_RUN  && wr_cnt <= chunk_n)
                   || (state == PTR_RUN && wr_cnt == 2'd0);
    assign wr_data  = (wr_cnt == 2'd0) ? addr_r[7:0] : (is_word && !pos) ? data_r[15:8] : data_r[7:0];

    assign data_valid = (state == RD_RUN) && rd_valid;
    assign data       = rd_data;
    assign data_last  = rd_last;
    assign rd_ready   = (state == RD_RUN) && data_ready;

    assign op_ok   = (result == R_OK);
    assign op_nack = (result == R_NACK);
    assign op_bus  = (result == R_BUS);
    assign op_poll = (result == R_POLL);
    assign op_polls = polls;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            result  <= R_NONE;
            op_r    <= '0;
            addr_r  <= '0;
            data_r  <= '0;
            len_r   <= '0;
            left    <= '0;
            wr_cnt  <= '0;
            pos     <= 1'b0;
            polls   <= '0;
            op_done <= 1'b0;
        end else begin
            op_done <= 1'b0;

            case (state)
                IDLE: begin
                    if (req_valid) begin
                        op_r   <= req_op;
                        addr_r <= req_addr;
                        data_r <= req_data;
                        len_r  <= (req_op == OP_RD_BYTE) ? LEN_W'(1) : req_len;
                        left   <= (req_op == OP_WR_WORD) ? 2'd2 : 2'd1;
                        wr_cnt <= '0;
                        pos    <= 1'b0;
                        polls  <= '0;
                        result <= R_NONE;
                        case (req_op)
                            OP_WR_BYTE, OP_WR_WORD: state <= WR_SEND;
                            OP_RD_CUR:              state <= RD_SEND;
                            default:                state <= PTR_SEND;
                        endcase
                    end
                end

                WR_SEND, PTR_SEND, RD_SEND, POLL_SEND: begin
                    if (cmd_valid && cmd_ready) begin
                        wr_cnt <= '0;
                        state  <= state_t'(state + 1'b1);
                    end
                end

                WR_RUN: begin
                    if (wr_valid && wr_ready) begin
                        wr_cnt <= wr_cnt + 1'b1;
                        if (wr_cnt != 2'd0) pos <= 1'b1;
                    end
                    if (done) begin
                        if (fail_bus) begin
                            result <= R_BUS;
                            state  <= FINISH;
                        end else if (fail_nack) begin
                            result <= R_NACK;
                            state  <= FINISH;
                        end else begin
                            state <= POLL_SEND;
                        end
                    end
                end

                POLL_RUN: begin
                    if (done) begin
                        if (fail_bus) begin
                            result <= R_BUS;
                            state  <= FINISH;
                        end else if (res_nack_addr) begin       // микросхема занята циклом записи
                            polls <= polls + 1'b1;
                            if (polls + 1'b1 >= 8'(POLL_MAX)) begin
                                result <= R_POLL;
                                state  <= FINISH;
                            end else begin
                                state <= POLL_SEND;
                            end
                        end else begin                          // ACK: запись закончена
                            addr_r <= addr_r + ADDR_W'(chunk_n);
                            left   <= left - chunk_n;
                            if (left == chunk_n) begin
                                result <= R_OK;
                                state  <= FINISH;
                            end else begin
                                state <= WR_SEND;
                            end
                        end
                    end
                end

                PTR_RUN: begin
                    if (wr_valid && wr_ready) wr_cnt <= wr_cnt + 1'b1;
                    if (done) begin
                        if (fail_bus) begin
                            result <= R_BUS;
                            state  <= FINISH;
                        end else if (fail_nack) begin
                            result <= R_NACK;
                            state  <= FINISH;
                        end else begin
                            state <= RD_SEND;
                        end
                    end
                end

                RD_RUN: begin
                    if (done) begin
                        if (fail_bus)       result <= R_BUS;
                        else if (fail_nack) result <= R_NACK;
                        else                result <= R_OK;
                        state <= FINISH;
                    end
                end

                FINISH: begin
                    op_done <= 1'b1;
                    state   <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
