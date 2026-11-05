// =============================================================================
// Проверка UART на плате, часть 2: команды МУ, пакеты МО -> МУ, индикация
//
// Приём (поток с ПК):
//   UART -> uart_pkt_parser -> (байты вне пакетов) -> uart_cmd_parser
//   - пакет 0xAA + 6 байт + XOR с верной CHK отправляется обратно (эхо пакета,
//     проверка парсера) через очередь на ECHO_DEPTH пакетов: она сглаживает
//     небольшую разницу скоростей ПК и ПЛИС на пачке пакетов; если очередь
//     полна, пакет теряется и считается ошибкой;
//   - пакет с неверной CHK или оборванный - только ошибка;
//   - 0x01 Старт: генератор test_pkt_gen начинает с n = 0 и выдаёт пакет
//     раз в PKT_PERIOD_CLKS; 0x02 Стоп: генератор останавливается;
//   - остальные байты - мусор, только счётчик.
// Передача (конвейер): три источника через uart_tx_mux, по приоритету:
//   0 - эхо верного пакета, 1 - пакеты генератора, 2 - строка MSG.
//
// Кнопки (после подавления дребезга, по нажатию):
//   KEY1 - один тестовый пакет от генератора;
//   KEY2 - отправить строку MSG;
//   KEY3 - обнулить счётчики индикации и погасить led[3];
//   KEY4 - следующий режим индикатора.
// Индикатор и светодиоды - см. uart_ind.sv:
//   «b XX» последний байт, «rNNN» принятые пакеты, «tNNN» переданные пакеты,
//   «ENNN» ошибки, «uNNN» мусор; led[0] приём, led[1] передача,
//   led[2] Старт, led[3] была ошибка.
// =============================================================================
module UART_top2 #(
    parameter int          CLK_FREQ        = 50_000_000,
    parameter int          BAUD            = 115_200,
    parameter int          PKT_PERIOD_CLKS = CLK_FREQ / 10,         // пакеты генератора: 100 мс
    parameter int          RX_TIMEOUT_CLKS = CLK_FREQ / 1000,       // обрыв пакета: 1 мс
    parameter int          DEB_CLKS        = CLK_FREQ / 1000 * 10,  // антидребезг 10 мс
    parameter int          DIGIT_HZ        = 1_000,
    parameter int          BLANK_CLKS      = 100,
    parameter int          FLASH_CLKS      = CLK_FREQ / 20,         // вспышка светодиода 50 мс
    parameter int          ECHO_DEPTH      = 4,                     // очередь эха, пакетов
    parameter int          MSG_LEN         = 11,
    parameter logic [8*MSG_LEN-1:0] MSG    = {"UART_top2", 8'h0D, 8'h0A},
    parameter bit          SEG_ACTIVE      = 1'b0,
    parameter bit          DIG_ACTIVE      = 1'b0,
    parameter bit          LED_ACTIVE      = 1'b0
) (
    input  logic       clk,
    input  logic       rst_n,      // кнопка сброса, нажата = 0
    input  logic [3:0] key,        // KEY1..KEY4, нажата = 0
    input  logic       uart_rx,    // <- TXD адаптера USB-TTL
    output logic       uart_tx,    // -> RXD адаптера USB-TTL
    output logic [7:0] seg,
    output logic [3:0] dig,
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
    // Кнопки
    // -------------------------------------------------------------------------
    logic [3:0] key_press;

    genvar i;   // Quartus Standard не понимает genvar внутри for

    generate
    for (i = 0; i < 4; i++) begin : g_key
        debounce #(
            .CLK_FREQ    (CLK_FREQ),
            .STABLE_CLKS (DEB_CLKS),
            .ACTIVE      (1'b0)
        ) u_deb (
            .clk       (clk),
            .rst_n     (rst_n_s),
            .btn       (key[i]),
            .pressed   (),
            .press_p   (key_press[i]),
            .release_p ()
        );
    end
    endgenerate

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
    // Парсеры: пакеты, затем команды из байтов вне пакетов
    // -------------------------------------------------------------------------
    logic [15:0] rx_meas, rx_filt, rx_err;
    logic        rx_pkt_valid;
    logic        rx_chk_error;
    logic        rx_timeout;
    logic [7:0]  pass_data;
    logic        pass_valid;

    uart_pkt_parser #(
        .TIMEOUT_CLKS (RX_TIMEOUT_CLKS)
    ) u_pkt_parser (
        .clk        (clk),
        .rst_n      (rst_n_s),
        .rx_data    (rx_data),
        .rx_valid   (rx_valid),
        .meas       (rx_meas),
        .filt       (rx_filt),
        .err        (rx_err),
        .pkt_valid  (rx_pkt_valid),
        .chk_error  (rx_chk_error),
        .timeout    (rx_timeout),
        .pass_data  (pass_data),
        .pass_valid (pass_valid),
        .busy       ()
    );

    logic cmd_start, cmd_stop, garbage, running;

    uart_cmd_parser u_cmd_parser (
        .clk       (clk),
        .rst_n     (rst_n_s),
        .data      (pass_data),
        .valid     (pass_valid),
        .cmd_start (cmd_start),
        .cmd_stop  (cmd_stop),
        .garbage   (garbage),
        .running   (running)
    );

    // -------------------------------------------------------------------------
    // Источники передачи: 0 - эхо пакета, 1 - генератор, 2 - строка
    // -------------------------------------------------------------------------
    localparam int SRC_ECHO = 0,
                   SRC_GEN  = 1,
                   SRC_MSG  = 2,
                   N_SRC    = 3;

    logic [N_SRC-1:0][7:0] src_data;
    logic [N_SRC-1:0]      src_valid, src_ready, src_last;

    // эхо: верный пакет -> очередь -> формирователь пакета
    logic [15:0] echo_meas, echo_filt, echo_err;
    logic        echo_valid, echo_ready;
    logic        fifo_in_ready;
    wire         echo_ovf = rx_pkt_valid && !fifo_in_ready;    // очередь полна

    pkt_fifo #(
        .WIDTH (48),
        .DEPTH (ECHO_DEPTH)
    ) u_echo_fifo (
        .clk       (clk),
        .rst_n     (rst_n_s),
        .in_data   ({rx_meas, rx_filt, rx_err}),
        .in_valid  (rx_pkt_valid),
        .in_ready  (fifo_in_ready),
        .out_data  ({echo_meas, echo_filt, echo_err}),
        .out_valid (echo_valid),
        .out_ready (echo_ready),
        .count     ()
    );

    uart_pkt_tx u_echo_tx (
        .clk      (clk),
        .rst_n    (rst_n_s),
        .meas     (echo_meas),
        .filt     (echo_filt),
        .err      (echo_err),
        .in_valid (echo_valid),
        .in_ready (echo_ready),
        .tx_data  (src_data [SRC_ECHO]),
        .tx_valid (src_valid[SRC_ECHO]),
        .tx_ready (src_ready[SRC_ECHO]),
        .tx_last  (src_last [SRC_ECHO])
    );

    // генератор тестовых пакетов: Старт/Стоп от МУ и KEY1
    logic [15:0] gen_meas, gen_filt, gen_err;
    logic        gen_valid, gen_ready;

    test_pkt_gen #(
        .CLK_FREQ    (CLK_FREQ),
        .PERIOD_CLKS (PKT_PERIOD_CLKS)
    ) u_gen (
        .clk     (clk),
        .rst_n   (rst_n_s),
        .run     (running),
        .single  (key_press[0]),
        .restart (cmd_start),
        .meas    (gen_meas),
        .filt    (gen_filt),
        .err     (gen_err),
        .valid   (gen_valid),
        .ready   (gen_ready)
    );

    uart_pkt_tx u_gen_tx (
        .clk      (clk),
        .rst_n    (rst_n_s),
        .meas     (gen_meas),
        .filt     (gen_filt),
        .err      (gen_err),
        .in_valid (gen_valid),
        .in_ready (gen_ready),
        .tx_data  (src_data [SRC_GEN]),
        .tx_valid (src_valid[SRC_GEN]),
        .tx_ready (src_ready[SRC_GEN]),
        .tx_last  (src_last [SRC_GEN])
    );

    // строка по KEY2
    uart_seq_tx #(
        .LEN (MSG_LEN),
        .SEQ (MSG)
    ) u_msg_tx (
        .clk      (clk),
        .rst_n    (rst_n_s),
        .start    (key_press[1]),
        .busy     (),
        .tx_data  (src_data [SRC_MSG]),
        .tx_valid (src_valid[SRC_MSG]),
        .tx_ready (src_ready[SRC_MSG]),
        .tx_last  (src_last [SRC_MSG])
    );

    uart_tx_mux #(
        .N (N_SRC)
    ) u_tx_mux (
        .clk       (clk),
        .rst_n     (rst_n_s),
        .in_data   (src_data),
        .in_valid  (src_valid),
        .in_last   (src_last),
        .in_ready  (src_ready),
        .out_data  (tx_data),
        .out_valid (tx_valid),
        .out_ready (tx_ready),
        .out_last  ()
    );

    // последний байт пакета (эхо или генератор) принят передатчиком
    wire [N_SRC-1:0] src_done = src_valid & src_ready & src_last;
    wire             tx_pkt   = src_done[SRC_ECHO] || src_done[SRC_GEN];

    // -------------------------------------------------------------------------
    // Индикация
    // -------------------------------------------------------------------------
    uart_ind #(
        .CLK_FREQ   (CLK_FREQ),
        .DIGIT_HZ   (DIGIT_HZ),
        .BLANK_CLKS (BLANK_CLKS),
        .FLASH_CLKS (FLASH_CLKS),
        .SEG_ACTIVE (SEG_ACTIVE),
        .DIG_ACTIVE (DIG_ACTIVE),
        .LED_ACTIVE (LED_ACTIVE)
    ) u_ind (
        .clk         (clk),
        .rst_n       (rst_n_s),
        .rx_byte     (rx_valid),
        .rx_data     (rx_data),
        .tx_byte     (tx_valid && tx_ready),
        .rx_pkt      (rx_pkt_valid),
        .tx_pkt      (tx_pkt),
        .garbage     (garbage),
        .err_frame   (rx_error),
        .err_chk     (rx_chk_error),
        .err_timeout (rx_timeout),
        .err_ovf     (echo_ovf),
        .running     (running),
        .clear       (key_press[2]),
        .next_mode   (key_press[3]),
        .seg         (seg),
        .dig         (dig),
        .led         (led)
    );

endmodule
