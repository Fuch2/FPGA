// =============================================================================
// Кольцевой буфер измерений в EEPROM (PROTOCOL.md, раздел 8.1)
//
// RECORDS записей по 2 байта (температура Q8.8, старший байт по меньшему адресу);
// запись n лежит по адресу 2n. Указатели записи wr_ptr и чтения rd_ptr хранятся
// в регистрах (PTR_W бит, переходят через 0 по кругу) и обнуляются командой START.
// Запросы к EEPROM выполняет eeprom_ctrl (интерфейс req_*, data_*, op_*).
//
// Команда (valid/ready), принимается в такте cmd_valid = 1 и cmd_ready = 1:
//   cmd_op = 0  START  обнулить указатели, счётчик записей и флаг переполнения;
//   cmd_op = 1  PUSH   дописать слово cmd_data в конец; если буфер полон, новая запись
//                      затирает самую старую (rd_ptr сдвигается вперёд) и ставится ovf;
//   cmd_op = 2  POP    прочитать самую старую запись в res_data; если записей нет -
//                      res_empty = 1 и обращения к EEPROM нет.
// Итог - импульс done; флаги и res_data держатся до следующей команды:
//   res_ok    - команда выполнена (для POP - res_data верно);
//   res_empty - POP при пустом буфере (res_ok = 0);
//   res_err   - EEPROM не ответила (NACK, тайм-аут, polling): указатели не изменены.
// Статус: ovf - после START буфер переполнялся (потеряны старые записи; сбрасывается
// командой START), count - сколько записей ещё не прочитано, wr_ptr / rd_ptr - указатели.
// busy - команда выполняется.
// =============================================================================
module eeprom_ring #(
    parameter int RECORDS = 512,                    // число записей, степень двойки
    parameter int ADDR_W  = 10,                     // разрядность адреса ячейки EEPROM (не меньше log2(RECORDS) + 1)
    parameter int LEN_W   = 8
) (
    input  logic                       clk,
    input  logic                       rst_n,       // асинхронный сброс, активный 0

    // команда
    input  logic                       cmd_valid,
    output logic                       cmd_ready,
    input  logic [1:0]                 cmd_op,
    input  logic [15:0]                cmd_data,

    // итог
    output logic                       done,
    output logic                       res_ok,
    output logic                       res_empty,
    output logic                       res_err,
    output logic [15:0]                res_data,

    // статус
    output logic                       busy,
    output logic                       ovf,
    output logic [$clog2(RECORDS):0]   count,
    output logic [$clog2(RECORDS)-1:0] wr_ptr,
    output logic [$clog2(RECORDS)-1:0] rd_ptr,

    // запросы к eeprom_ctrl
    output logic                       req_valid,
    input  logic                       req_ready,
    output logic [2:0]                 req_op,
    output logic [ADDR_W-1:0]          req_addr,
    output logic [15:0]                req_data,
    output logic [LEN_W-1:0]           req_len,
    input  logic                       data_valid,
    output logic                       data_ready,
    input  logic [7:0]                 data,
    input  logic                       op_done,
    input  logic                       op_ok
);

    localparam logic [1:0] CMD_START = 2'd0,
                           CMD_PUSH  = 2'd1,
                           CMD_POP   = 2'd2;

    localparam logic [2:0] OP_WR_WORD = 3'd1,
                           OP_RD_SEQ  = 3'd3;

    localparam int PTR_W = $clog2(RECORDS);

    typedef enum logic [2:0] {
        IDLE,
        PUSH_REQ, PUSH_RUN,     // запись слова в EEPROM
        POP_REQ,  POP_RUN,      // чтение двух байт из EEPROM
        FINISH
    } state_t;

    state_t      state;
    logic [15:0] push_word;
    logic        byte_idx;          // 0 - ждём старший байт записи, 1 - младший
    logic [7:0]  hi_byte, lo_byte;
    logic        got_lo;            // принят и младший байт

    wire full = (count == (PTR_W + 1)'(RECORDS));

    assign cmd_ready  = (state == IDLE);
    assign busy       = (state != IDLE);

    assign req_valid  = (state == PUSH_REQ) || (state == POP_REQ);
    assign req_op     = (state == PUSH_REQ) ? OP_WR_WORD : OP_RD_SEQ;
    assign req_addr   = (state == PUSH_REQ) ? ADDR_W'({wr_ptr, 1'b0}) : ADDR_W'({rd_ptr, 1'b0});
    assign req_data   = push_word;
    assign req_len    = LEN_W'(2);
    assign data_ready = (state == POP_RUN);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            push_word <= '0;
            byte_idx  <= 1'b0;
            hi_byte   <= '0;
            lo_byte   <= '0;
            got_lo    <= 1'b0;
            wr_ptr    <= '0;
            rd_ptr    <= '0;
            count     <= '0;
            ovf       <= 1'b0;
            done      <= 1'b0;
            res_ok    <= 1'b0;
            res_empty <= 1'b0;
            res_err   <= 1'b0;
            res_data  <= '0;
        end else begin
            done <= 1'b0;

            case (state)
                IDLE: begin
                    if (cmd_valid) begin
                        push_word <= cmd_data;
                        byte_idx  <= 1'b0;
                        got_lo    <= 1'b0;
                        res_ok    <= 1'b0;
                        res_empty <= 1'b0;
                        res_err   <= 1'b0;
                        case (cmd_op)
                            CMD_START: begin
                                wr_ptr <= '0;
                                rd_ptr <= '0;
                                count  <= '0;
                                ovf    <= 1'b0;
                                res_ok <= 1'b1;
                                state  <= FINISH;
                            end
                            CMD_PUSH: state <= PUSH_REQ;
                            CMD_POP: begin
                                if (count == '0) begin
                                    res_empty <= 1'b1;
                                    state     <= FINISH;
                                end else begin
                                    state <= POP_REQ;
                                end
                            end
                            default: begin              // неизвестная команда: ошибка
                                res_err <= 1'b1;
                                state   <= FINISH;
                            end
                        endcase
                    end
                end

                PUSH_REQ, POP_REQ: begin
                    if (req_valid && req_ready) state <= state_t'(state + 1'b1);
                end

                PUSH_RUN: begin
                    if (op_done) begin
                        if (op_ok) begin
                            wr_ptr <= wr_ptr + 1'b1;
                            if (full) begin             // затираем самую старую запись
                                rd_ptr <= rd_ptr + 1'b1;
                                ovf    <= 1'b1;
                            end else begin
                                count <= count + 1'b1;
                            end
                            res_ok <= 1'b1;
                        end else begin
                            res_err <= 1'b1;
                        end
                        state <= FINISH;
                    end
                end

                POP_RUN: begin
                    if (data_valid && data_ready) begin
                        byte_idx <= 1'b1;
                        if (!byte_idx) begin
                            hi_byte <= data;
                        end else begin
                            lo_byte <= data;
                            got_lo  <= 1'b1;
                        end
                    end
                    if (op_done) begin
                        if (op_ok && got_lo) begin
                            rd_ptr   <= rd_ptr + 1'b1;
                            count    <= count - 1'b1;
                            res_data <= {hi_byte, lo_byte};
                            res_ok   <= 1'b1;
                        end else begin
                            res_err <= 1'b1;
                        end
                        state <= FINISH;
                    end
                end

                FINISH: begin
                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
