// =============================================================================
// Формирователь пакета данных МО -> МУ (PROTOCOL.md, раздел 5.2)
//
// Пакет 8 байт, слова старшим байтом вперёд:
//   0      SYNC (0xAA)
//   1, 2   MEAS - измеренное значение, Q8.8
//   3, 4   FILT - отфильтрованное значение, Q8.8
//   5, 6   ERR  - ошибка, Q8.8 (вычисляет источник данных, здесь не меняется)
//   7      CHK  = XOR байтов 1..6
//
// Вход (valid/ready): слова meas, filt, err принимаются в такте, где
//   in_valid = 1 и in_ready = 1; in_ready = 1, только пока пакет не передаётся.
// Выход (valid/ready, к передатчику UART или к uart_tx_mux):
//   байт tx_data уходит в такте, где tx_valid = 1 и tx_ready = 1;
//   tx_last = 1 на последнем байте пакета (CHK).
// =============================================================================
module uart_pkt_tx #(
    parameter logic [7:0] SYNC = 8'hAA      // синхробайт
) (
    input  logic        clk,
    input  logic        rst_n,      // асинхронный сброс, активный 0

    input  logic [15:0] meas,
    input  logic [15:0] filt,
    input  logic [15:0] err,
    input  logic        in_valid,
    output logic        in_ready,

    output logic [7:0]  tx_data,
    output logic        tx_valid,
    input  logic        tx_ready,
    output logic        tx_last
);

    localparam int PKT_LEN = 8;

    typedef enum logic {IDLE, SEND} state_t;

    state_t                         state;
    logic [8*PKT_LEN-1:0]           shreg;      // байты пакета, текущий - старший
    logic [$clog2(PKT_LEN)-1:0]     byte_idx;   // номер текущего байта

    wire [7:0] chk = meas[15:8] ^ meas[7:0] ^ filt[15:8] ^ filt[7:0] ^ err[15:8] ^ err[7:0];

    assign in_ready = (state == IDLE);
    assign tx_valid = (state == SEND);
    assign tx_data  = shreg[8*PKT_LEN-1 -: 8];
    assign tx_last  = (byte_idx == PKT_LEN - 1);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            shreg    <= '0;
            byte_idx <= '0;
        end else begin
            case (state)
                IDLE: begin
                    if (in_valid) begin
                        shreg    <= {SYNC, meas, filt, err, chk};
                        byte_idx <= '0;
                        state    <= SEND;
                    end
                end

                SEND: begin
                    if (tx_ready) begin
                        shreg    <= shreg << 8;
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
