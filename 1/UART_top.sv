// =============================================================================
// Проверка UART на плате: эхо
//   каждый принятый байт отправляется обратно (через FIFO на 16 байт,
//   чтобы не терять данные, если ПК передаёт чуть быстрее, чем отвечает ПЛИС).
//
// Светодиоды:
//   led[0] - мигает раз в секунду: прошивка загружена и работает
//   led[1] - вспышка при приёме байта
//   led[2] - вспышка при передаче байта
//   led[3] - ошибка (ошибка кадра или переполнение FIFO), горит до сброса
// =============================================================================
module UART_top #(
    parameter int CLK_FREQ   = 50_000_000,
    parameter int BAUD       = 115_200,
    parameter bit LED_ACTIVE = 1'b0         // 0 - светодиод горит при 0 на выводе
) (
    input  logic       clk,
    input  logic       rst_n,               // кнопка сброса, нажата = 0
    input  logic       uart_rx,             // <- TXD адаптера USB-TTL
    output logic       uart_tx,             // -> RXD адаптера USB-TTL
    output logic [3:0] led
);

    // -------------------------------------------------------------------------
    // Синхронизация сброса: асинхронная установка, синхронное снятие
    // -------------------------------------------------------------------------
    logic [1:0] rst_sync;
    logic       rst_n_s;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_sync <= 2'b00;
        else        rst_sync <= {rst_sync[0], 1'b1};
    end

    assign rst_n_s = rst_sync[1];

    // -------------------------------------------------------------------------
    // UART
    // -------------------------------------------------------------------------
    logic [7:0] tx_data;
    logic       tx_valid;
    logic       tx_ready;
    logic [7:0] rx_data;
    logic       rx_valid;
    logic       rx_error;

    UART #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) u_uart (
        .clk      (clk),
        .rst_n    (rst_n_s),
        .tx_data  (tx_data),
        .tx_valid (tx_valid),
        .tx_ready (tx_ready),
        .tx       (uart_tx),
        .rx       (uart_rx),
        .rx_data  (rx_data),
        .rx_valid (rx_valid),
        .rx_error (rx_error)
    );

    // -------------------------------------------------------------------------
    // FIFO 16 x 8: приёмник -> передатчик
    // -------------------------------------------------------------------------
    localparam int AW = 4;

    logic [7:0]  fifo_mem [2**AW];
    logic [AW:0] wr_ptr, rd_ptr;             // старший бит - признак круга

    wire fifo_empty = (wr_ptr == rd_ptr);
    wire fifo_full  = (wr_ptr[AW] != rd_ptr[AW]) && (wr_ptr[AW-1:0] == rd_ptr[AW-1:0]);
    wire fifo_wr    = rx_valid && !fifo_full;
    wire fifo_rd    = tx_valid && tx_ready;
    wire overflow   = rx_valid && fifo_full;

    always_ff @(posedge clk) begin
        if (fifo_wr)
            fifo_mem[wr_ptr[AW-1:0]] <= rx_data;
    end

    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
        end else begin
            if (fifo_wr) wr_ptr <= wr_ptr + 1'b1;
            if (fifo_rd) rd_ptr <= rd_ptr + 1'b1;
        end
    end

    assign tx_valid = !fifo_empty;
    assign tx_data  = fifo_mem[rd_ptr[AW-1:0]];

    // -------------------------------------------------------------------------
    // Светодиоды
    // -------------------------------------------------------------------------
    localparam int HALF_SEC = CLK_FREQ / 2;
    localparam int FLASH    = CLK_FREQ / 20;     // вспышка 50 мс

    logic [$clog2(HALF_SEC+1)-1:0] hb_cnt;
    logic [$clog2(FLASH+1)-1:0]    rx_flash, tx_flash;
    logic                          hb, err;
    logic [3:0]                    led_on;

    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s) begin
            hb_cnt   <= '0;
            hb       <= 1'b0;
            rx_flash <= '0;
            tx_flash <= '0;
            err      <= 1'b0;
        end else begin
            if (hb_cnt == HALF_SEC - 1) begin
                hb_cnt <= '0;
                hb     <= ~hb;
            end else begin
                hb_cnt <= hb_cnt + 1'b1;
            end

            if (rx_valid)          rx_flash <= FLASH;
            else if (rx_flash != 0) rx_flash <= rx_flash - 1'b1;

            if (fifo_rd)           tx_flash <= FLASH;
            else if (tx_flash != 0) tx_flash <= tx_flash - 1'b1;

            if (rx_error || overflow) err <= 1'b1;
        end
    end

    assign led_on = {err, (tx_flash != 0), (rx_flash != 0), hb};
    assign led    = LED_ACTIVE ? led_on : ~led_on;

endmodule
