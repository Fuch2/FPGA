// =============================================================================
// Проверка индикации на плате: ШИМ + динамическая индикация + тестовые данные
//
// Кнопки (после подавления дребезга, срабатывают по нажатию):
//   KEY1 - яркость ниже, KEY2 - яркость выше (8 ступеней, после сброса максимум);
//   KEY3 - следующий режим тестовых данных (0 -> 1 -> 2 -> 3 -> 0);
//   KEY4 - пауза / продолжение.
// Режимы (см. test_seq.sv):
//   0 - проверка сегментов a..g, dp во всех разрядах, затем «8.» по одному разряду;
//   1 - десятичный счётчик; 2 - шестнадцатеричный счётчик; 3 - таблица.
//
// Светодиоды:
//   led[0] - мигает раз в секунду: прошивка загружена и работает;
//   led[1], led[2] - номер режима (led[1] - младший бит);
//   led[3] - пауза.
// =============================================================================
module IND_top #(
    parameter int CLK_FREQ   = 50_000_000,
    parameter int DIGIT_HZ   = 1_000,                     // смена разрядов, Гц
    parameter int BLANK_CLKS = 100,                       // гашение между разрядами (2 мкс)
    parameter int PWM_FREQ   = 16_000,                    // ШИМ яркости, Гц
    parameter int DEB_CLKS   = CLK_FREQ / 1000 * 10,      // антидребезг 10 мс
    parameter int SLOW_CLKS  = CLK_FREQ,                  // шаг режимов 0, 3 (1 с)
    parameter int FAST_CLKS  = CLK_FREQ / 10,             // шаг режимов 1, 2 (0.1 с)
    parameter int HB_CLKS    = CLK_FREQ / 2,              // полупериод мигания led[0]
    parameter bit SEG_ACTIVE = 1'b0,
    parameter bit DIG_ACTIVE = 1'b0,
    parameter bit LED_ACTIVE = 1'b0
) (
    input  logic       clk,
    input  logic       rst_n,      // кнопка сброса, нажата = 0
    input  logic [3:0] key,        // KEY1..KEY4, нажата = 0
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
    // Яркость, режим, пауза
    // -------------------------------------------------------------------------
    logic [2:0] level;           // ступень яркости 0..7
    logic [1:0] mode;
    logic       run;
    logic [7:0] duty;

    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s) begin
            level <= 3'd7;
            mode  <= 2'd0;
            run   <= 1'b1;
        end else begin
            if (key_press[0] && level != 3'd0) level <= level - 1'b1;
            if (key_press[1] && level != 3'd7) level <= level + 1'b1;
            if (key_press[2])                  mode  <= mode + 1'b1;
            if (key_press[3])                  run   <= ~run;
        end
    end

    // ступени яркости примерно вдвое друг от друга: 2^(level+1) - 1 из 255
    assign duty = 8'((16'd2 << level) - 1'b1);

    logic pwm_on;

    pwm #(
        .CLK_FREQ (CLK_FREQ),
        .PWM_FREQ (PWM_FREQ),
        .WIDTH    (8)
    ) u_pwm (
        .clk          (clk),
        .rst_n        (rst_n_s),
        .duty         (duty),
        .pwm_out      (pwm_on),
        .period_start ()
    );

    // -------------------------------------------------------------------------
    // Тестовые данные -> индикатор
    // -------------------------------------------------------------------------
    logic [15:0] t_hex;
    logic [3:0]  t_dp, t_blank, t_raw;
    logic [31:0] t_seg_raw;
    logic        t_valid;

    test_seq #(
        .CLK_FREQ  (CLK_FREQ),
        .SLOW_CLKS (SLOW_CLKS),
        .FAST_CLKS (FAST_CLKS)
    ) u_seq (
        .clk     (clk),
        .rst_n   (rst_n_s),
        .mode    (mode),
        .run     (run),
        .hex     (t_hex),
        .dp      (t_dp),
        .blank   (t_blank),
        .raw     (t_raw),
        .seg_raw (t_seg_raw),
        .valid   (t_valid)
    );

    seg7_dynamic #(
        .CLK_FREQ   (CLK_FREQ),
        .N          (4),
        .DIGIT_HZ   (DIGIT_HZ),
        .BLANK_CLKS (BLANK_CLKS),
        .SEG_ACTIVE (SEG_ACTIVE),
        .DIG_ACTIVE (DIG_ACTIVE)
    ) u_disp (
        .clk     (clk),
        .rst_n   (rst_n_s),
        .hex     (t_hex),
        .dp      (t_dp),
        .blank   (t_blank),
        .raw     (t_raw),
        .seg_raw (t_seg_raw),
        .valid   (t_valid),
        .en      (pwm_on),
        .seg     (seg),
        .dig     (dig)
    );

    // -------------------------------------------------------------------------
    // Светодиоды
    // -------------------------------------------------------------------------
    logic [$clog2(HB_CLKS+1)-1:0] hb_cnt;
    logic                         hb;
    logic [3:0]                   led_on;

    always_ff @(posedge clk or negedge rst_n_s) begin
        if (!rst_n_s) begin
            hb_cnt <= '0;
            hb     <= 1'b0;
        end else if (hb_cnt == HB_CLKS - 1) begin
            hb_cnt <= '0;
            hb     <= ~hb;
        end else begin
            hb_cnt <= hb_cnt + 1'b1;
        end
    end

    assign led_on = {~run, mode, hb};
    assign led    = LED_ACTIVE ? led_on : ~led_on;

endmodule
