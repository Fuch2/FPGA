// =============================================================================
// Формирователь ответа МИ -> МО (PROTOCOL.md, раздел 6.1)
//
// Ответ - 4 байта: STATUS, D_H, D_L, CHK, где CHK = ~(STATUS ^ D_H ^ D_L).
// Слово {status, data} защёлкивается в момент frame_start (спад CS_N) и
// передаётся в этом кадре байт за байтом; изменение status и data после
// frame_start в текущем кадре не видно (протокол: «слейв защёлкивает слово
// в момент спада CS_N»).
//
// Выход (valid/ready, к spi_slave):
//   байт tx_data уходит в такте, где tx_valid = 1 и tx_ready = 1;
//   после 4 байт tx_valid = 0 до следующего frame_start.
// =============================================================================
module spi_resp_tx (
    input  logic        clk,
    input  logic        rst_n,        // асинхронный сброс, активный 0

    input  logic        frame_start,
    input  logic [7:0]  status,
    input  logic [15:0] data,

    output logic [7:0]  tx_data,
    output logic        tx_valid,
    input  logic        tx_ready
);

    localparam int RESP_LEN = 4;

    typedef enum logic {IDLE, SEND} state_t;

    state_t                     state;
    logic [8*RESP_LEN-1:0]      shreg;      // слово ответа, текущий байт - старший
    logic [$clog2(RESP_LEN):0]  byte_idx;   // сколько байт передано

    wire [7:0] chk = ~(status ^ data[15:8] ^ data[7:0]);

    assign tx_valid = (state == SEND);
    assign tx_data  = shreg[8*RESP_LEN-1 -: 8];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= IDLE;
            shreg    <= '0;
            byte_idx <= '0;
        end else if (frame_start) begin
            shreg    <= {status, data, chk};
            byte_idx <= '0;
            state    <= SEND;
        end else if (state == SEND && tx_ready) begin
            shreg    <= shreg << 8;
            byte_idx <= byte_idx + 1'b1;
            if (byte_idx == RESP_LEN - 1)
                state <= IDLE;
        end
    end

endmodule
