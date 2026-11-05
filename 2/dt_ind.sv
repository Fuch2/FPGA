// =============================================================================
// Индикация чтения ДТ (плата B): температура в градусах и обмен по I2C
//
// Входы-события - импульсы на 1 такт:
//   ev_start, ev_ack, ev_nack - события шины от мастера (START; ведомый подтвердил /
//                               не подтвердил адрес или байт);
//   temp_valid + temp_data    - новое значение температуры (Q8.8) от dt_reader;
//   read_fail                 - чтение не удалось.
// Управление: clear - обнулить счётчики и погасить led[3]; next_mode - следующий
// режим; show_temp / show_src (импульсы) - перейти на температуру / на экран источника.
// src_model, gen_mode - состояние источника для экрана источника.
//
// Режимы индикатора (dig[3] - буква или первая цифра, режим 0 - все 4 цифры):
//   0  «XX.X»  температура в °C: «25.4», « 5.3», «-5.3», «-25.4», «125.0»;
//              « Err» - последнее чтение не удалось, «----» - значения ещё не было
//   1  «XXXX»  последнее принятое значение, hex (Q8.8: 0x1960 = 25.375 °C)
//   2  «rNNN»  удачных чтений;  режим 3: «ENNN» - неудачных чтений
//   4  «Sx y»  источник (третий разряд пуст): x = 0 реальный датчик, 1 имитатор; y - режим имитатора
//              (0 константа, 1 линейный, 2 пила, 3 шум)
// Счётчики десятичные (BCD), 0..999, после 999 идёт 000.
//
// Светодиоды:  led[0] - вспышка при START;  led[1] - вспышка при ACK ведомого;
//              led[2] - вспышка при NACK ведомого;  led[3] - было неудачное чтение (до clear).
// =============================================================================
module dt_ind #(
    parameter int CLK_FREQ    = 50_000_000,     // частота clk, Гц
    parameter int DIGIT_HZ    = 1_000,          // смена разрядов индикатора, Гц
    parameter int BLANK_CLKS  = 100,            // гашение между разрядами, тактов
    parameter int FLASH_CLKS  = CLK_FREQ / 20,  // длительность вспышки светодиода (50 мс)
    parameter bit SEG_ACTIVE  = 1'b0,
    parameter bit DIG_ACTIVE  = 1'b0,
    parameter bit LED_ACTIVE  = 1'b0
) (
    input  logic        clk,
    input  logic        rst_n,          // асинхронный сброс, активный 0

    input  logic        ev_start,
    input  logic        ev_ack,
    input  logic        ev_nack,
    input  logic        temp_valid,
    input  logic [15:0] temp_data,
    input  logic        read_fail,

    input  logic        src_model,
    input  logic [1:0]  gen_mode,

    input  logic        clear,
    input  logic        next_mode,
    input  logic        show_temp,
    input  logic        show_src,

    output logic [7:0]  seg,
    output logic [3:0]  dig,
    output logic [3:0]  led
);

    typedef enum logic [2:0] {
        M_TEMP = 3'd0,
        M_HEX  = 3'd1,
        M_OK   = 3'd2,
        M_FAIL = 3'd3,
        M_SRC  = 3'd4
    } mode_t;

    localparam int N_MODES = 5;

    // буквы и знаки, порядок сегментов {dp, g, f, e, d, c, b, a}
    localparam logic [7:0] SEG_MINUS = 8'h40,
                           LET_E     = 8'h79,   // E
                           LET_R     = 8'h50,   // r
                           LET_S     = 8'h6D;   // S (как 5)

    // десятичный счётчик 000..999: +1 с переносом
    function automatic logic [11:0] bcd_inc(input logic [11:0] c);
        logic [11:0] r;
        logic        carry;
        carry = 1'b1;
        for (int i = 0; i < 3; i++) begin
            if (carry && c[4*i +: 4] == 4'd9) begin
                r[4*i +: 4] = 4'd0;
            end else begin
                r[4*i +: 4] = c[4*i +: 4] + carry;
                carry       = 1'b0;
            end
        end
        return r;
    endfunction

    // -------------------------------------------------------------------------
    // Перевод температуры в десятичные цифры
    // -------------------------------------------------------------------------
    logic        fmt_valid, fmt_neg;
    logic [15:0] fmt_bcd;

    temp_fmt u_fmt (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (temp_valid),
        .temp_data (temp_data),
        .busy      (),
        .out_valid (fmt_valid),
        .neg       (fmt_neg),
        .bcd       (fmt_bcd)
    );

    // -------------------------------------------------------------------------
    // Показываемые значения, счётчики, режим
    // -------------------------------------------------------------------------
    logic [15:0] shown_raw;                     // последнее принятое значение
    logic [15:0] shown_bcd;                     // его цифры
    logic        shown_neg;
    logic        have_value;                    // было хотя бы одно значение
    logic        last_ok;                       // последнее чтение удалось
    logic [11:0] cnt_ok, cnt_fail;
    logic        fail_seen;
    mode_t       mode;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shown_raw  <= '0;
            shown_bcd  <= '0;
            shown_neg  <= 1'b0;
            have_value <= 1'b0;
            last_ok    <= 1'b1;
            cnt_ok     <= '0;
            cnt_fail   <= '0;
            fail_seen  <= 1'b0;
            mode       <= M_TEMP;
        end else begin
            if (next_mode)  mode <= (mode == mode_t'(N_MODES - 1)) ? M_TEMP : mode_t'(mode + 1'b1);
            if (show_temp)  mode <= M_TEMP;
            if (show_src)   mode <= M_SRC;

            if (temp_valid) begin
                shown_raw <= temp_data;
                last_ok   <= 1'b1;
            end
            if (fmt_valid) begin
                shown_bcd  <= fmt_bcd;
                shown_neg  <= fmt_neg;
                have_value <= 1'b1;
            end

            if (clear) begin
                cnt_ok    <= '0;
                cnt_fail  <= '0;
                fail_seen <= 1'b0;
            end else begin
                if (temp_valid) cnt_ok <= bcd_inc(cnt_ok);
                if (read_fail) begin
                    cnt_fail  <= bcd_inc(cnt_fail);
                    fail_seen <= 1'b1;
                end
            end
            if (read_fail) last_ok <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // Индикатор
    // -------------------------------------------------------------------------
    logic [15:0] number;
    logic [3:0]  d_blank, d_raw, d_dp;
    logic [31:0] raw_seg;                       // {dig[3], dig[2], dig[1], dig[0]} по 8 бит

    wire [3:0] t_hund  = shown_bcd[15:12];
    wire [3:0] t_tens  = shown_bcd[11:8];
    wire       big_tens = (t_hund != 4'd0) || (t_tens != 4'd0);

    always_comb begin
        number  = '0;
        d_blank = 4'b0000;
        d_raw   = 4'b0000;
        d_dp    = 4'b0000;
        raw_seg = '0;
        case (mode)
            M_TEMP: begin
                if (!last_ok) begin                             // « Err»
                    d_raw   = 4'b0111;
                    d_blank = 4'b1000;
                    raw_seg = {8'h00, LET_E, LET_R, LET_R};
                end else if (!have_value) begin                 // «----»
                    d_raw   = 4'b1111;
                    raw_seg = {4{SEG_MINUS}};
                end else begin
                    number = shown_bcd;
                    d_dp   = 4'b0010;                           // точка после единиц
                    if (shown_neg) begin
                        // минус стоит перед первой значащей цифрой
                        if (big_tens) begin
                            d_raw   = 4'b1000;
                            raw_seg = {SEG_MINUS, 24'h0};
                        end else begin
                            d_raw   = 4'b0100;
                            d_blank = 4'b1000;
                            raw_seg = {8'h00, SEG_MINUS, 16'h0};
                        end
                    end else begin
                        d_blank = {t_hund == 4'd0, (t_hund == 4'd0) && (t_tens == 4'd0), 2'b00};
                    end
                end
            end
            M_HEX:  number = shown_raw;
            M_OK: begin
                number  = {4'h0, cnt_ok};
                d_raw   = 4'b1000;
                raw_seg = {LET_R, 24'h0};
            end
            M_FAIL: begin
                number  = {4'h0, cnt_fail};
                d_raw   = 4'b1000;
                raw_seg = {LET_E, 24'h0};
            end
            default: begin                                      // M_SRC: «S x y»
                number  = {4'h0, 3'b000, src_model, 4'h0, 2'b00, gen_mode};
                d_raw   = 4'b1000;
                d_blank = 4'b0010;
                raw_seg = {LET_S, 24'h0};
            end
        endcase
    end

    seg7_dynamic #(
        .CLK_FREQ   (CLK_FREQ),
        .N          (4),
        .DIGIT_HZ   (DIGIT_HZ),
        .BLANK_CLKS (BLANK_CLKS),
        .SEG_ACTIVE (SEG_ACTIVE),
        .DIG_ACTIVE (DIG_ACTIVE)
    ) u_disp (
        .clk     (clk),
        .rst_n   (rst_n),
        .hex     (number),
        .dp      (d_dp),
        .blank   (d_blank),
        .raw     (d_raw),
        .seg_raw (raw_seg),
        .valid   (1'b1),                                // данные обновляются каждый такт
        .en      (1'b1),
        .seg     (seg),
        .dig     (dig)
    );

    // -------------------------------------------------------------------------
    // Светодиоды
    // -------------------------------------------------------------------------
    localparam int FLASH_W = $clog2(FLASH_CLKS + 1);

    logic [FLASH_W-1:0] start_flash, ack_flash, nack_flash;
    logic [3:0]         led_on;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_flash <= '0;
            ack_flash   <= '0;
            nack_flash  <= '0;
        end else begin
            if (ev_start)                start_flash <= FLASH_W'(FLASH_CLKS);
            else if (start_flash != 0)   start_flash <= start_flash - 1'b1;

            if (ev_ack)                  ack_flash <= FLASH_W'(FLASH_CLKS);
            else if (ack_flash != 0)     ack_flash <= ack_flash - 1'b1;

            if (ev_nack)                 nack_flash <= FLASH_W'(FLASH_CLKS);
            else if (nack_flash != 0)    nack_flash <= nack_flash - 1'b1;
        end
    end

    assign led_on = {fail_seen, (nack_flash != 0), (ack_flash != 0), (start_flash != 0)};
    assign led    = LED_ACTIVE ? led_on : ~led_on;

endmodule
