// =============================================================================
// Проверка SPI на плате A (МО = мастер): команды Старт, Измерение, Стоп в плату B
//
// Кнопки (после подавления дребезга, по нажатию):
//   KEY1 - «Старт»;  KEY2 - «Измерение»;  KEY3 - «Стоп»;
//   KEY4 - следующий режим индикатора. Обнуление счётчиков - кнопкой RESET.
// Каждая команда - две транзакции (spi_cmd_tx): кадр с командой, пауза, кадр NOP
// с ответом МИ: {STATUS, D_H, D_L, CHK}. Нажатия во время выполнения команды
// игнорируются и считаются ошибкой.
//
// Индикатор и светодиоды - см. spi_ind.sv:
//   «XXXX» слово D из ответа на «Измерение» (у заглушки МИ: счётчик 0001, 0002, ...),
//   «c XX» код последней команды, «S XX» STATUS из ответа,
//   «tNNN» кадров начато, «rNNN» верных ответов, «ENNN» ошибок;
//   led[0] передача, led[1] приём верного ответа, led[2] RUN в STATUS ответа,
//   led[3] была ошибка (CHK / MARK неверны, например МИ не подключена).
// =============================================================================
module SPI_topA #(
    parameter int CLK_FREQ    = 50_000_000,
    parameter int SCK_FREQ    = 1_000_000,
    parameter int CS_SETUP_NS = 1_000,
    parameter int CS_GAP_NS   = 2_000,
    parameter int T_RESP_US   = 2_000,                  // пауза до ответа на «Измерение»
    parameter int T_ACK_US    = 1_000,                  // пауза до ответа на «Старт» / «Стоп»
    parameter int DEB_CLKS    = CLK_FREQ / 1000 * 10,   // антидребезг 10 мс
    parameter int DIGIT_HZ    = 1_000,
    parameter int BLANK_CLKS  = 100,
    parameter int FLASH_CLKS  = CLK_FREQ / 20,          // вспышка светодиода 50 мс
    parameter bit SEG_ACTIVE  = 1'b0,
    parameter bit DIG_ACTIVE  = 1'b0,
    parameter bit LED_ACTIVE  = 1'b0
) (
    input  logic       clk,
    input  logic       rst_n,       // кнопка сброса, нажата = 0
    input  logic [3:0] key,         // KEY1..KEY4, нажата = 0

    output logic       spi_cs_n,
    output logic       spi_sck,
    output logic       spi_mosi,
    input  logic       spi_miso,

    output logic [7:0] seg,
    output logic [3:0] dig,
    output logic [3:0] led
);

    localparam int STATUS_RUN   = 0;     // номера битов STATUS (PROTOCOL.md, раздел 6.3)
    localparam int STATUS_VALID = 1;

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
    // Конвейер команд и SPI-мастер
    // -------------------------------------------------------------------------
    logic [7:0]  m_tx_data, m_rx_data;
    logic        m_tx_valid, m_tx_ready, m_tx_last;
    logic        m_rx_valid, m_busy;

    logic        req_lost;
    logic        rsp_valid, rsp_final, rsp_ok;
    logic [7:0]  rsp_cmd, rsp_status;
    logic [15:0] rsp_data;

    spi_cmd_tx #(
        .CLK_FREQ  (CLK_FREQ),
        .T_RESP_US (T_RESP_US),
        .T_ACK_US  (T_ACK_US)
    ) u_cmd_tx (
        .clk         (clk),
        .rst_n       (rst_n_s),
        .req_start   (key_press[0]),
        .req_measure (key_press[1]),
        .req_stop    (key_press[2]),
        .busy        (),
        .req_lost    (req_lost),
        .tx_data     (m_tx_data),
        .tx_valid    (m_tx_valid),
        .tx_ready    (m_tx_ready),
        .tx_last     (m_tx_last),
        .rx_data     (m_rx_data),
        .rx_valid    (m_rx_valid),
        .link_busy   (m_busy),
        .rsp_valid   (rsp_valid),
        .rsp_final   (rsp_final),
        .rsp_cmd     (rsp_cmd),
        .rsp_status  (rsp_status),
        .rsp_data    (rsp_data),
        .rsp_ok      (rsp_ok)
    );

    spi_master #(
        .CLK_FREQ    (CLK_FREQ),
        .SCK_FREQ    (SCK_FREQ),
        .CS_SETUP_NS (CS_SETUP_NS),
        .CS_GAP_NS   (CS_GAP_NS)
    ) u_master (
        .clk      (clk),
        .rst_n    (rst_n_s),
        .tx_data  (m_tx_data),
        .tx_valid (m_tx_valid),
        .tx_ready (m_tx_ready),
        .tx_last  (m_tx_last),
        .rx_data  (m_rx_data),
        .rx_valid (m_rx_valid),
        .busy     (m_busy),
        .spi_cs_n (spi_cs_n),
        .spi_sck  (spi_sck),
        .spi_mosi (spi_mosi),
        .spi_miso (spi_miso)
    );

    // -------------------------------------------------------------------------
    // Индикация
    // -------------------------------------------------------------------------
    logic m_busy_q;
    logic run_seen;                   // RUN из последнего верного ответа на команду

    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s) begin
            m_busy_q <= 1'b0;
            run_seen <= 1'b0;
        end else begin
            m_busy_q <= m_busy;
            if (rsp_valid && rsp_final && rsp_ok)
                run_seen <= rsp_status[STATUS_RUN];
        end
    end

    wire answer_ok = rsp_valid && rsp_final && rsp_ok;

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
        .tx_frame    (m_busy && !m_busy_q),
        .rx_frame    (rsp_valid && rsp_ok),
        .err         ((rsp_valid && !rsp_ok) || req_lost),
        .cmd_we      (rsp_valid && rsp_final),
        .info_cmd    (rsp_cmd),
        .status_we   (answer_ok),
        .info_status (rsp_status),
        .data_we     (answer_ok && rsp_status[STATUS_VALID]),
        .info_data   (rsp_data),
        .running     (run_seen),
        .clear       (1'b0),
        .next_mode   (key_press[3]),
        .seg         (seg),
        .dig         (dig),
        .led         (led)
    );

endmodule
