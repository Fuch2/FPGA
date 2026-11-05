// =============================================================================
// Проверка SPI на плате B (МИ = слейв): приём команд от платы A, заглушка вместо ДТ
//
// Цепочка: spi_slave -> spi_cmd_parser (Старт / Измерение / Стоп) -> spi_mi_stub
// (флаги RUN / VALID / EMPTY / BUSY и счётчик 0001, 0002, ... вместо температуры)
// -> spi_resp_tx (слово ответа STATUS, D_H, D_L, CHK) -> spi_slave -> MISO.
// Вывод MISO переходит в Z при CS_N = 1.
//
// Кнопки (после подавления дребезга, по нажатию):
//   KEY1 - следующий режим индикатора;  KEY2 - обнулить счётчики и погасить led[3].
// Индикатор и светодиоды - см. spi_ind.sv:
//   «XXXX» последнее значение счётчика (слово D), «c XX» последняя команда
//   (A1 / A2 / A3), «S XX» STATUS последнего ответа, «tNNN» отданных ответов,
//   «rNNN» верных кадров команд, «ENNN» ошибок (кадр не 4 байта, неизвестная
//   команда, не хватило данных для ответа);
//   led[0] передача, led[1] приём верного кадра, led[2] RUN, led[3] была ошибка.
// =============================================================================
module SPI_topB #(
    parameter int CLK_FREQ   = 50_000_000,
    parameter int MEAS_US    = 500,                     // время «чтения EEPROM» заглушкой
    parameter int DEB_CLKS   = CLK_FREQ / 1000 * 10,    // антидребезг 10 мс
    parameter int DIGIT_HZ   = 1_000,
    parameter int BLANK_CLKS = 100,
    parameter int FLASH_CLKS = CLK_FREQ / 20,           // вспышка светодиода 50 мс
    parameter bit SEG_ACTIVE = 1'b0,
    parameter bit DIG_ACTIVE = 1'b0,
    parameter bit LED_ACTIVE = 1'b0
) (
    input  logic       clk,
    input  logic       rst_n,       // кнопка сброса, нажата = 0
    input  logic [3:0] key,         // KEY1..KEY4, нажата = 0

    input  logic       spi_cs_n,
    input  logic       spi_sck,
    input  logic       spi_mosi,
    output logic       spi_miso,

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
    // SPI-слейв
    // -------------------------------------------------------------------------
    logic [7:0] s_rx_data, s_tx_data;
    logic       s_rx_valid, s_tx_valid, s_tx_ready, s_tx_underrun;
    logic       frame_start, frame_end, frame_short;
    logic       miso, miso_oe;

    spi_slave u_slave (
        .clk         (clk),
        .rst_n       (rst_n_s),
        .spi_cs_n    (spi_cs_n),
        .spi_sck     (spi_sck),
        .spi_mosi    (spi_mosi),
        .spi_miso    (miso),
        .spi_miso_oe (miso_oe),
        .rx_data     (s_rx_data),
        .rx_valid    (s_rx_valid),
        .tx_data     (s_tx_data),
        .tx_valid    (s_tx_valid),
        .tx_ready    (s_tx_ready),
        .tx_underrun (s_tx_underrun),
        .frame_start (frame_start),
        .frame_end   (frame_end),
        .frame_short (frame_short)
    );

    assign spi_miso = miso_oe ? miso : 1'bz;

    // -------------------------------------------------------------------------
    // Команды, заглушка МИ, ответ
    // -------------------------------------------------------------------------
    logic       cmd_start, cmd_measure, cmd_stop, cmd_nop, cmd_unknown, frame_err;
    logic [7:0] last_cmd;

    spi_cmd_parser u_parser (
        .clk         (clk),
        .rst_n       (rst_n_s),
        .frame_start (frame_start),
        .frame_end   (frame_end),
        .frame_short (frame_short),
        .rx_data     (s_rx_data),
        .rx_valid    (s_rx_valid),
        .cmd_start   (cmd_start),
        .cmd_measure (cmd_measure),
        .cmd_stop    (cmd_stop),
        .cmd_nop     (cmd_nop),
        .cmd_unknown (cmd_unknown),
        .frame_err   (frame_err),
        .last_cmd    (last_cmd)
    );

    logic [7:0]  mi_status;
    logic [15:0] mi_data;
    logic        mi_running, meas_done;

    spi_mi_stub #(
        .CLK_FREQ (CLK_FREQ),
        .MEAS_US  (MEAS_US)
    ) u_stub (
        .clk         (clk),
        .rst_n       (rst_n_s),
        .cmd_start   (cmd_start),
        .cmd_measure (cmd_measure),
        .cmd_stop    (cmd_stop),
        .status      (mi_status),
        .data        (mi_data),
        .running     (mi_running),
        .meas_done   (meas_done)
    );

    spi_resp_tx u_resp (
        .clk         (clk),
        .rst_n       (rst_n_s),
        .frame_start (frame_start),
        .status      (mi_status),
        .data        (mi_data),
        .tx_data     (s_tx_data),
        .tx_valid    (s_tx_valid),
        .tx_ready    (s_tx_ready)
    );

    // -------------------------------------------------------------------------
    // Индикация
    // -------------------------------------------------------------------------
    logic [7:0] sent_status;          // STATUS, отданный в текущем (последнем) кадре

    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s)         sent_status <= '0;
        else if (frame_start) sent_status <= mi_status;
    end

    wire cmd_valid = cmd_start || cmd_measure || cmd_stop;

    spi_ind #(
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
        .tx_frame    (frame_end),
        .rx_frame    (cmd_valid || cmd_nop),
        .err         (frame_err || cmd_unknown || s_tx_underrun),
        .cmd_we      (cmd_valid),
        .info_cmd    (last_cmd),
        .status_we   (frame_end),
        .info_status (sent_status),
        .data_we     (meas_done),
        .info_data   (mi_data),
        .running     (mi_running),
        .clear       (key_press[1]),
        .next_mode   (key_press[0]),
        .seg         (seg),
        .dig         (dig),
        .led         (led)
    );

endmodule
