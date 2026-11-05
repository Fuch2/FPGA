// =============================================================================
// UART: приёмник и передатчик, формат кадра 8N1
//   1 старт-бит (0), 8 бит данных (младший бит первым), без чётности, 1 стоп-бит (1)
//
// Интерфейс передатчика (valid/ready):
//   данные tx_data принимаются в такте, где tx_valid = 1 и tx_ready = 1.
// Интерфейс приёмника:
//   rx_valid - импульс на 1 такт, rx_data действительны и держатся до следующего байта;
//   rx_error - импульс на 1 такт, если стоп-бит оказался равен 0 (ошибка кадра).
// =============================================================================
module UART #(
    parameter int CLK_FREQ = 50_000_000,  // частота clk, Гц
    parameter int BAUD     = 115_200      // скорость обмена, бит/с
) (
    input  logic       clk,
    input  logic       rst_n,     // асинхронный сброс, активный 0

    // передатчик
    input  logic [7:0] tx_data,
    input  logic       tx_valid,
    output logic       tx_ready,
    output logic       tx,        // линия TX

    // приёмник
    input  logic       rx,        // линия RX
    output logic [7:0] rx_data,
    output logic       rx_valid,
    output logic       rx_error
);

    // число тактов clk на один бит (с округлением)
    localparam int CLKS_PER_BIT = (CLK_FREQ + BAUD / 2) / BAUD;

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk   (clk),
        .rst_n (rst_n),
        .data  (tx_data),
        .valid (tx_valid),
        .ready (tx_ready),
        .tx    (tx)
    );

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk   (clk),
        .rst_n (rst_n),
        .rx    (rx),
        .data  (rx_data),
        .valid (rx_valid),
        .error (rx_error)
    );

endmodule
