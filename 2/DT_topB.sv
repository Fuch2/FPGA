// =============================================================================
// Проверка ДТ на плате B: чтение температуры LM75A, индикация в градусах
//
// Цепочка: опрос (таймер или KEY1) -> dt_reader -> dt_bus (i2c_master + выбор шины,
// шина I2C: SCL PIN_99, SDA PIN_98, либо внутренний имитатор dt_model)
// -> dt_ind (температура на индикаторе, светодиоды обмена).
//
// Кнопки (после подавления дребезга, по нажатию):
//   KEY1 - прочитать температуру сейчас и показать её в градусах;
//   KEY2 - источник: реальный датчик <-> имитатор dt_model (обнуляет счётчики);
//   KEY3 - следующий режим индикатора (температура, hex, удачные, неудачные, источник);
//   KEY4 - следующий режим имитатора (0 константа, 1 линейный рост, 2 пила, 3 шум).
// Опрос по таймеру раз в POLL_MS миллисекунд, первый - сразу после сброса.
// Светодиоды: led[0] START, led[1] ACK ведомого, led[2] NACK ведомого,
//             led[3] было неудачное чтение (гаснет при смене источника KEY2).
// Подробности режимов индикатора - в dt_ind.sv, параметры имитатора - ниже.
// =============================================================================
module DT_topB #(
    parameter int         CLK_FREQ        = 50_000_000,
    parameter int         SCL_FREQ        = 100_000,
    parameter int         TIMEOUT_US      = 1_000,
    parameter logic [6:0] DT_ADDR         = 7'h48,        // LM75A (A2..A0 = 0)
    parameter int         POLL_MS         = 500,          // период опроса датчика, мс
    parameter int         MODEL_UPDATE_MS = 500,          // период обновления температуры имитатора, мс
    parameter int         DEB_CLKS        = CLK_FREQ / 1000 * 10,   // антидребезг 10 мс
    parameter int         DIGIT_HZ        = 1_000,
    parameter int         BLANK_CLKS      = 100,
    parameter int         FLASH_CLKS      = CLK_FREQ / 20,          // вспышка светодиода 50 мс
    parameter bit         SEG_ACTIVE      = 1'b0,
    parameter bit         DIG_ACTIVE      = 1'b0,
    parameter bit         LED_ACTIVE      = 1'b0
) (
    input  logic       clk,
    input  logic       rst_n,       // кнопка сброса, нажата = 0
    input  logic [3:0] key,         // KEY1..KEY4, нажата = 0

    inout  wire        i2c_scl,
    inout  wire        i2c_sda,

    output logic [7:0] seg,
    output logic [3:0] dig,
    output logic [3:0] led
);

    localparam int LEN_W      = 8;
    localparam int POLL_CLKS  = CLK_FREQ / 1000 * POLL_MS;
    localparam int POLL_W     = $clog2(POLL_CLKS + 1);

    // параметры имитатора в шагах 0.125 °C: пила 20.0 .. 30.0 °C с шагом 0.5 °C, шум до +1.875 °C
    localparam logic signed [10:0] GEN_LO         = 11'sd160;
    localparam logic signed [10:0] GEN_HI         = 11'sd240;
    localparam logic        [10:0] GEN_STEP       = 11'd4;
    localparam logic        [7:0]  GEN_NOISE_MASK = 8'h0F;

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
    // Источник и режим имитатора, таймер опроса
    // -------------------------------------------------------------------------
    logic             use_model;
    logic [1:0]       gen_mode;
    logic [POLL_W-1:0] poll_cnt;
    logic             poll_p, start_read;

    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s) begin
            use_model <= 1'b0;
            gen_mode  <= 2'd2;                  // пила
            poll_cnt  <= POLL_W'(POLL_CLKS - 1);// первый опрос сразу после сброса
            poll_p    <= 1'b0;
        end else begin
            if (key_press[1]) use_model <= !use_model;
            if (key_press[3]) gen_mode  <= gen_mode + 1'b1;

            poll_p <= (poll_cnt == POLL_CLKS - 1);
            if (poll_cnt == POLL_CLKS - 1) poll_cnt <= '0;
            else                           poll_cnt <= poll_cnt + 1'b1;
        end
    end

    assign start_read = poll_p || key_press[0];

    // -------------------------------------------------------------------------
    // Чтение датчика, мастер и шина
    // -------------------------------------------------------------------------
    logic             cmd_valid, cmd_ready, cmd_read, cmd_stop;
    logic [6:0]       cmd_addr;
    logic [LEN_W-1:0] cmd_len;
    logic [7:0]       wr_data, rd_data;
    logic             wr_valid, wr_ready, rd_valid, rd_ready, rd_last;
    logic             done, res_nack_addr, res_nack_data, res_timeout, res_arb;
    logic [LEN_W-1:0] res_len;
    logic             ev_start, ev_stop, ev_ack, ev_nack;

    logic             busy, temp_valid, read_fail;
    logic [15:0]      temp_data;

    dt_reader #(
        .LEN_W   (LEN_W),
        .DT_ADDR (DT_ADDR)
    ) u_reader (
        .clk           (clk),
        .rst_n         (rst_n_s),
        .start_read    (start_read),
        .busy          (busy),
        .temp_valid    (temp_valid),
        .temp_data     (temp_data),
        .read_fail     (read_fail),
        .cmd_valid     (cmd_valid),
        .cmd_ready     (cmd_ready),
        .cmd_addr      (cmd_addr),
        .cmd_read      (cmd_read),
        .cmd_len       (cmd_len),
        .cmd_stop      (cmd_stop),
        .wr_data       (wr_data),
        .wr_valid      (wr_valid),
        .wr_ready      (wr_ready),
        .rd_data       (rd_data),
        .rd_valid      (rd_valid),
        .rd_ready      (rd_ready),
        .rd_last       (rd_last),
        .done          (done),
        .res_nack_addr (res_nack_addr),
        .res_nack_data (res_nack_data),
        .res_timeout   (res_timeout),
        .res_arb       (res_arb)
    );

    dt_bus #(
        .CLK_FREQ        (CLK_FREQ),
        .SCL_FREQ        (SCL_FREQ),
        .TIMEOUT_US      (TIMEOUT_US),
        .LEN_W           (LEN_W),
        .MODEL_ADDR      (DT_ADDR),
        .MODEL_UPDATE_MS (MODEL_UPDATE_MS)
    ) u_bus (
        .clk            (clk),
        .rst_n          (rst_n_s),
        .use_model      (use_model),
        .gen_restart    (key_press[1]),
        .gen_mode       (gen_mode),
        .gen_lo         (GEN_LO),
        .gen_hi         (GEN_HI),
        .gen_step       (GEN_STEP),
        .gen_noise_mask (GEN_NOISE_MASK),
        .cmd_valid      (cmd_valid),
        .cmd_ready      (cmd_ready),
        .cmd_addr       (cmd_addr),
        .cmd_read       (cmd_read),
        .cmd_len        (cmd_len),
        .cmd_stop       (cmd_stop),
        .wr_data        (wr_data),
        .wr_valid       (wr_valid),
        .wr_ready       (wr_ready),
        .rd_data        (rd_data),
        .rd_valid       (rd_valid),
        .rd_ready       (rd_ready),
        .rd_last        (rd_last),
        .done           (done),
        .res_len        (res_len),
        .res_nack_addr  (res_nack_addr),
        .res_nack_data  (res_nack_data),
        .res_timeout    (res_timeout),
        .res_arb        (res_arb),
        .ev_start       (ev_start),
        .ev_stop        (ev_stop),
        .ev_ack         (ev_ack),
        .ev_nack        (ev_nack),
        .i2c_scl        (i2c_scl),
        .i2c_sda        (i2c_sda)
    );

    // -------------------------------------------------------------------------
    // Индикация
    // -------------------------------------------------------------------------
    dt_ind #(
        .CLK_FREQ   (CLK_FREQ),
        .DIGIT_HZ   (DIGIT_HZ),
        .BLANK_CLKS (BLANK_CLKS),
        .FLASH_CLKS (FLASH_CLKS),
        .SEG_ACTIVE (SEG_ACTIVE),
        .DIG_ACTIVE (DIG_ACTIVE),
        .LED_ACTIVE (LED_ACTIVE)
    ) u_ind (
        .clk        (clk),
        .rst_n      (rst_n_s),
        .ev_start   (ev_start),
        .ev_ack     (ev_ack),
        .ev_nack    (ev_nack),
        .temp_valid (temp_valid),
        .temp_data  (temp_data),
        .read_fail  (read_fail),
        .src_model  (use_model),
        .gen_mode   (gen_mode),
        .clear      (key_press[1]),
        .next_mode  (key_press[2]),
        .show_temp  (key_press[0]),
        .show_src   (key_press[1] || key_press[3]),
        .seg        (seg),
        .dig        (dig),
        .led        (led)
    );

endmodule
