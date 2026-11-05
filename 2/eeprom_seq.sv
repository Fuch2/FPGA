// =============================================================================
// Сценарии проверки EEPROM на плате B: кольцо, прямая запись и чтение
//
// По импульсу run выполняется сценарий script через eeprom_store: команды кольца
// (ring_*) или прямые запросы (raw_*). Все записи ведут себя как измерения: значение
// gen_val (GEN_BASE, GEN_STEP, ... после каждой удачной записи) - «температура» Q8.8.
//   0  START     кольцо: обнулить указатели; курсор и значение gen_val - в начало
//   1  PUSH      кольцо: записать gen_val в конец
//   2  POP       кольцо: прочитать самую старую запись (info_empty = 1, если записей нет)
//   3  FILL      кольцо: RECORDS + FILL_EXTRA записей подряд (проверка переполнения)
//   4  CLEAR     прямая запись 0xFFFF в CLEAR_WORDS слов с адреса 0 (стереть начало кольца)
//   5  WR_BYTE   прямая запись младшего байта gen_val по адресу cursor, курсор + 1
//   6  WR_WORD   прямая запись слова gen_val по адресу cursor, курсор + 2
//   7  RD_BYTE   прямое чтение байта по адресу cursor, курсор + 1
//   8  RD_WORD   прямое чтение слова (2 байта) по адресу cursor, курсор + 2
//   9  RD_SEQ    последовательное чтение SEQ_LEN байт с адреса cursor, курсор + SEQ_LEN;
//                в info_data - два последних прочитанных байта
// Сценарий прерывается на первой ошибке EEPROM (error = 1 до следующего run).
//
// Результат последней операции: info_addr (адрес ячейки), info_data (записанное или
// прочитанное слово; при чтении байта - 0x00XX), info_empty, polls_last (сколько
// опросов ACK заняла последняя запись - длина цикла записи микросхемы).
// События для индикации: wr_p, rd_p (начата запись / чтение), err_p (ошибка), все
// импульсы на 1 такт. busy - сценарий выполняется, finished - импульс в конце.
// =============================================================================
module eeprom_seq #(
    parameter int          RECORDS     = 512,
    parameter int          ADDR_W      = 10,
    parameter int          LEN_W       = 8,
    parameter int          FILL_EXTRA  = 8,                // на сколько записей FILL больше кольца
    parameter int          CLEAR_WORDS = 8,                // сколько слов стирает CLEAR
    parameter int          SEQ_LEN     = 4,                // длина RD_SEQ, байт
    parameter logic [15:0] GEN_BASE    = 16'h1960,         // первое значение (25.375 C)
    parameter logic [15:0] GEN_STEP    = 16'h0020          // шаг (0.125 C)
) (
    input  logic                       clk,
    input  logic                       rst_n,           // асинхронный сброс, активный 0

    input  logic                       run,
    input  logic [3:0]                 script,
    output logic                       busy,
    output logic                       finished,
    output logic                       error,

    output logic [ADDR_W-1:0]          info_addr,
    output logic [15:0]                info_data,
    output logic                       info_empty,
    output logic [ADDR_W-1:0]          cursor,
    output logic [7:0]                 polls_last,
    output logic                       wr_p,
    output logic                       rd_p,
    output logic                       err_p,

    // кольцо
    output logic                       ring_cmd_valid,
    input  logic                       ring_cmd_ready,
    output logic [1:0]                 ring_cmd_op,
    output logic [15:0]                ring_cmd_data,
    input  logic                       ring_done,
    input  logic                       ring_ok,
    input  logic                       ring_empty,
    input  logic                       ring_err,
    input  logic [15:0]                ring_data,
    input  logic [$clog2(RECORDS)-1:0] ring_wr_ptr,
    input  logic [$clog2(RECORDS)-1:0] ring_rd_ptr,

    // прямой доступ
    output logic                       raw_valid,
    input  logic                       raw_ready,
    output logic [2:0]                 raw_op,
    output logic [ADDR_W-1:0]          raw_addr,
    output logic [15:0]                raw_data,
    output logic [LEN_W-1:0]           raw_len,
    input  logic                       raw_rd_valid,
    input  logic [7:0]                 raw_rd_data,
    output logic                       raw_rd_ready,
    input  logic                       raw_done,
    input  logic                       raw_ok,
    input  logic [7:0]                 raw_polls
);

    localparam logic [3:0] SC_START   = 4'd0,
                           SC_PUSH    = 4'd1,
                           SC_POP     = 4'd2,
                           SC_FILL    = 4'd3,
                           SC_CLEAR   = 4'd4,
                           SC_WR_BYTE = 4'd5,
                           SC_WR_WORD = 4'd6,
                           SC_RD_BYTE = 4'd7,
                           SC_RD_WORD = 4'd8,
                           SC_RD_SEQ  = 4'd9;

    localparam logic [1:0] CMD_START = 2'd0,
                           CMD_PUSH  = 2'd1,
                           CMD_POP   = 2'd2;

    localparam logic [2:0] OP_WR_BYTE = 3'd0,
                           OP_WR_WORD = 3'd1,
                           OP_RD_BYTE = 3'd2,
                           OP_RD_SEQ  = 3'd3;

    localparam int ITER_W = $clog2(RECORDS + FILL_EXTRA + 1);

    typedef enum logic [1:0] {IDLE, ISSUE, WAIT, FINISH} state_t;

    state_t             state;
    logic [3:0]         script_r;
    logic [ITER_W-1:0]  iter;               // сколько операций сценария уже выполнено
    logic [ITER_W-1:0]  iters;              // сколько операций в сценарии
    logic [15:0]        gen_val;
    logic [15:0]        rd_shift;           // последние два прочитанных байта
    logic [ADDR_W-1:0]  op_addr;            // адрес текущей операции

    wire is_ring  = (script_r == SC_START) || (script_r == SC_PUSH) || (script_r == SC_POP) || (script_r == SC_FILL);
    wire is_write = (script_r == SC_PUSH) || (script_r == SC_FILL) || (script_r == SC_CLEAR)
                 || (script_r == SC_WR_BYTE) || (script_r == SC_WR_WORD);
    wire is_read  = (script_r == SC_POP) || (script_r == SC_RD_BYTE) || (script_r == SC_RD_WORD)
                 || (script_r == SC_RD_SEQ);

    always_comb begin
        case (script_r)
            SC_START: ring_cmd_op = CMD_START;
            SC_POP:   ring_cmd_op = CMD_POP;
            default:  ring_cmd_op = CMD_PUSH;
        endcase
        case (script_r)
            SC_CLEAR:   raw_op = OP_WR_WORD;
            SC_WR_BYTE: raw_op = OP_WR_BYTE;
            SC_WR_WORD: raw_op = OP_WR_WORD;
            SC_RD_BYTE: raw_op = OP_RD_BYTE;
            default:    raw_op = OP_RD_SEQ;
        endcase
        case (script_r)
            SC_RD_SEQ: raw_len = LEN_W'(SEQ_LEN);
            default:   raw_len = LEN_W'(2);
        endcase
        iters = ITER_W'(1);
        if (script_r == SC_FILL)  iters = ITER_W'(RECORDS + FILL_EXTRA);
        if (script_r == SC_CLEAR) iters = ITER_W'(CLEAR_WORDS);
    end

    assign ring_cmd_data = gen_val;
    assign raw_data      = (script_r == SC_CLEAR) ? 16'hFFFF : gen_val;
    assign raw_addr      = (script_r == SC_CLEAR) ? ADDR_W'({iter, 1'b0}) : cursor;
    assign raw_rd_ready  = 1'b1;
    assign ring_cmd_valid = (state == ISSUE) && is_ring;
    assign raw_valid      = (state == ISSUE) && !is_ring;
    assign busy           = (state != IDLE);

    wire issued = (ring_cmd_valid && ring_cmd_ready) || (raw_valid && raw_ready);
    wire done_p = is_ring ? ring_done : raw_done;
    wire fail   = is_ring ? ring_err : !raw_ok;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            script_r   <= '0;
            iter       <= '0;
            gen_val    <= GEN_BASE;
            rd_shift   <= '0;
            op_addr    <= '0;
            cursor     <= '0;
            info_addr  <= '0;
            info_data  <= '0;
            info_empty <= 1'b0;
            polls_last <= '0;
            finished   <= 1'b0;
            error      <= 1'b0;
            wr_p       <= 1'b0;
            rd_p       <= 1'b0;
            err_p      <= 1'b0;
        end else begin
            finished <= 1'b0;
            wr_p     <= 1'b0;
            rd_p     <= 1'b0;
            err_p    <= 1'b0;

            if (raw_rd_valid) rd_shift <= {rd_shift[7:0], raw_rd_data};

            case (state)
                IDLE: begin
                    if (run) begin
                        script_r <= script;
                        iter     <= '0;
                        error    <= 1'b0;
                        state    <= ISSUE;
                    end
                end

                ISSUE: begin
                    if (issued) begin
                        wr_p     <= is_write;
                        rd_p     <= is_read;
                        rd_shift <= '0;
                        case (script_r)
                            SC_PUSH, SC_FILL: op_addr <= ADDR_W'({ring_wr_ptr, 1'b0});
                            SC_POP:           op_addr <= ADDR_W'({ring_rd_ptr, 1'b0});
                            SC_START:         op_addr <= '0;
                            default:          op_addr <= raw_addr;
                        endcase
                        state <= WAIT;
                    end
                end

                WAIT: begin
                    if (done_p) begin
                        if (fail) begin
                            error <= 1'b1;
                            err_p <= 1'b1;
                            state <= FINISH;
                        end else begin
                            info_addr <= op_addr;
                            if (is_write) polls_last <= raw_polls;
                            case (script_r)
                                SC_START: begin
                                    cursor     <= '0;
                                    gen_val    <= GEN_BASE;
                                    info_data  <= '0;
                                    info_empty <= 1'b0;
                                end
                                SC_PUSH, SC_FILL: begin
                                    info_data  <= gen_val;
                                    info_empty <= 1'b0;
                                    gen_val    <= gen_val + GEN_STEP;
                                end
                                SC_POP: begin
                                    info_empty <= ring_empty;
                                    if (!ring_empty) info_data <= ring_data;
                                end
                                SC_CLEAR: begin
                                    info_data  <= 16'hFFFF;
                                    info_empty <= 1'b0;
                                end
                                SC_WR_BYTE: begin
                                    info_data  <= {8'h00, gen_val[7:0]};
                                    info_empty <= 1'b0;
                                    gen_val    <= gen_val + GEN_STEP;
                                    cursor     <= cursor + 1'b1;
                                end
                                SC_WR_WORD: begin
                                    info_data  <= gen_val;
                                    info_empty <= 1'b0;
                                    gen_val    <= gen_val + GEN_STEP;
                                    cursor     <= cursor + ADDR_W'(2);
                                end
                                SC_RD_BYTE: begin
                                    info_data  <= rd_shift;
                                    info_empty <= 1'b0;
                                    cursor     <= cursor + 1'b1;
                                end
                                SC_RD_WORD: begin
                                    info_data  <= rd_shift;
                                    info_empty <= 1'b0;
                                    cursor     <= cursor + ADDR_W'(2);
                                end
                                default: begin              // SC_RD_SEQ
                                    info_data  <= rd_shift;
                                    info_empty <= 1'b0;
                                    cursor     <= cursor + ADDR_W'(SEQ_LEN);
                                end
                            endcase
                            iter  <= iter + 1'b1;
                            state <= (iter + 1'b1 == iters) ? FINISH : ISSUE;
                        end
                    end
                end

                FINISH: begin
                    finished <= 1'b1;
                    state    <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
