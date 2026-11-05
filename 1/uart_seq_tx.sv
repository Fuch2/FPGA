// =============================================================================
// Конвейер передатчика: отправка заданной последовательности байт
//
// Последовательность задаётся параметрами: LEN байт в векторе SEQ, первый
// байт - старший, поэтому строку можно записать литералом:
//   .LEN(4), .SEQ("TEST")                     -> 'T', 'E', 'S', 'T'
//   .LEN(3), .SEQ({8'hAA, 8'h01, 8'h02})      -> AA 01 02
//
// Импульс start запускает передачу всей последовательности; пока она идёт
// (busy = 1), новые start игнорируются.
// Выход (valid/ready, к передатчику UART или к uart_tx_mux):
//   байт tx_data уходит в такте, где tx_valid = 1 и tx_ready = 1;
//   tx_last = 1 на последнем байте последовательности.
// =============================================================================
module uart_seq_tx #(
    parameter int             LEN = 4,          // длина последовательности, байт
    parameter logic [8*LEN-1:0] SEQ = "TEST"    // байты, первый - старший
) (
    input  logic       clk,
    input  logic       rst_n,      // асинхронный сброс, активный 0

    input  logic       start,
    output logic       busy,

    output logic [7:0] tx_data,
    output logic       tx_valid,
    input  logic       tx_ready,
    output logic       tx_last
);

    localparam int IDX_W = (LEN > 1) ? $clog2(LEN) : 1;

    typedef enum logic {IDLE, SEND} state_t;

    state_t           state;
    logic [IDX_W-1:0] byte_idx;     // номер текущего байта

    assign busy     = (state == SEND);
    assign tx_valid = (state == SEND);
    assign tx_data  = SEQ[8*(LEN-1-byte_idx) +: 8];
    assign tx_last  = (byte_idx == LEN - 1);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            byte_idx <= '0;
        end else begin
            case (state)
                IDLE: begin
                    byte_idx <= '0;
                    if (start)
                        state <= SEND;
                end

                SEND: begin
                    if (tx_ready) begin
                        byte_idx <= byte_idx + 1'b1;
                        if (tx_last)
                            state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
