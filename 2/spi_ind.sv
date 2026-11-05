// =============================================================================
// Индикация обмена по SPI (общая для платы A - МО и платы B - МИ)
//
// Входы-события - импульсы на 1 такт:
//   tx_frame - начат или передан кадр (A: кадр ушёл мастеру на линию;
//              B: ответ отдан мастеру);
//   rx_frame - принят верный кадр (A: ответ МИ с верной CHK;
//              B: кадр команды правильной длины с известной командой);
//   err      - ошибка (A: CHK / MARK неверны или запрос потерян;
//              B: кадр неверной длины, неизвестная команда, нехватка данных).
// Данные для показа (каждое защёлкивается по своему разрешению):
//   cmd_we    - запомнить info_cmd (A: код отправленной команды; B: принятая
//               команда Старт / Измерение / Стоп, NOP не считается);
//   status_we - запомнить info_status (A: STATUS из ответа на команду;
//               B: STATUS, отданный МО в последнем кадре);
//   data_we   - запомнить info_data (слово D; A: из ответа на Измерение,
//               B: новое значение, полученное по Измерению).
// Уровень running - RUN: A - из последнего STATUS, B - таймер МИ запущен.
// Управление (импульсы): clear - обнулить счётчики и погасить led[3];
//   next_mode - следующий режим индикатора.
//
// Режимы индикатора (dig[3] - буква, dig[2..0] - число; режим 0 - 4 цифры):
//   0  «XXXX» - слово D в hex (последнее значение, пришедшее от МИ);
//   1  «c XX» - код последней команды (hex);
//   2  «S XX» - последний STATUS (hex);
//   3  «tNNN» - число переданных кадров;
//   4  «rNNN» - число принятых верных кадров;
//   5  «ENNN» - число ошибок.
// Счётчики десятичные (BCD), 0..999, после 999 идёт 000.
//
// Светодиоды:
//   led[0] - вспышка при передаче кадра;  led[1] - вспышка при приёме верного кадра;
//   led[2] - running;                      led[3] - была ошибка (до clear).
// =============================================================================
module spi_ind #(
    parameter int CLK_FREQ   = 50_000_000,      // частота clk, Гц
    parameter int DIGIT_HZ   = 1_000,           // смена разрядов индикатора, Гц
    parameter int BLANK_CLKS = 100,             // гашение между разрядами, тактов
    parameter int FLASH_CLKS = CLK_FREQ / 20,   // длительность вспышки светодиода (50 мс)
    parameter bit SEG_ACTIVE = 1'b0,
    parameter bit DIG_ACTIVE = 1'b0,
    parameter bit LED_ACTIVE = 1'b0
) (
    input  logic        clk,
    input  logic        rst_n,       // асинхронный сброс, активный 0

    input  logic        tx_frame,
    input  logic        rx_frame,
    input  logic        err,

    input  logic        cmd_we,
    input  logic [7:0]  info_cmd,
    input  logic        status_we,
    input  logic [7:0]  info_status,
    input  logic        data_we,
    input  logic [15:0] info_data,
    input  logic        running,

    input  logic        clear,
    input  logic        next_mode,

    output logic [7:0]  seg,
    output logic [3:0]  dig,
    output logic [3:0]  led
);

    localparam int N_MODES = 6;

    typedef enum logic [2:0] {
        M_DATA   = 3'd0,
        M_CMD    = 3'd1,
        M_STATUS = 3'd2,
        M_TX     = 3'd3,
        M_RX     = 3'd4,
        M_ERR    = 3'd5
    } mode_t;

    // буквы для разряда dig[3], порядок сегментов {dp, g, f, e, d, c, b, a}
    localparam logic [7:0] LET_C = 8'h58,   // c
                           LET_S = 8'h6D,   // S (как 5)
                           LET_T = 8'h78,   // t
                           LET_R = 8'h50,   // r
                           LET_E = 8'h79;   // E

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
    // Счётчики и запомненные значения
    // -------------------------------------------------------------------------
    logic [15:0] last_data;
    logic [7:0]  last_cmd, last_status;
    logic [11:0] cnt_tx, cnt_rx, cnt_err;
    logic        err_seen;
    mode_t       mode;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_data   <= '0;
            last_cmd    <= '0;
            last_status <= '0;
            cnt_tx      <= '0;
            cnt_rx      <= '0;
            cnt_err     <= '0;
            err_seen    <= 1'b0;
            mode        <= M_DATA;
        end else begin
            if (next_mode)
                mode <= (mode == N_MODES - 1) ? M_DATA : mode_t'(mode + 1'b1);

            if (clear) begin
                last_data   <= '0;
                last_cmd    <= '0;
                last_status <= '0;
                cnt_tx      <= '0;
                cnt_rx      <= '0;
                cnt_err     <= '0;
                err_seen    <= 1'b0;
            end else begin
                if (data_we)   last_data   <= info_data;
                if (cmd_we)    last_cmd    <= info_cmd;
                if (status_we) last_status <= info_status;
                if (tx_frame) cnt_tx <= bcd_inc(cnt_tx);
                if (rx_frame) cnt_rx <= bcd_inc(cnt_rx);
                if (err) begin
                    cnt_err  <= bcd_inc(cnt_err);
                    err_seen <= 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Индикатор
    // -------------------------------------------------------------------------
    logic [15:0] number;
    logic [7:0]  letter;
    logic [3:0]  d_blank;
    logic [3:0]  d_raw;

    always_comb begin
        letter  = '0;
        d_blank = 4'b0000;
        d_raw   = 4'b0000;
        number  = '0;
        case (mode)
            M_DATA:   number = last_data;
            M_CMD: begin
                letter  = LET_C;
                number  = {8'h00, last_cmd};
                d_raw   = 4'b1000;
                d_blank = 4'b0100;              // dig[2] погашен: «c XX»
            end
            M_STATUS: begin
                letter  = LET_S;
                number  = {8'h00, last_status};
                d_raw   = 4'b1000;
                d_blank = 4'b0100;
            end
            M_TX: begin
                letter = LET_T;
                number = {4'h0, cnt_tx};
                d_raw  = 4'b1000;
            end
            M_RX: begin
                letter = LET_R;
                number = {4'h0, cnt_rx};
                d_raw  = 4'b1000;
            end
            default: begin
                letter = LET_E;
                number = {4'h0, cnt_err};
                d_raw  = 4'b1000;
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
        .dp      (4'b0000),
        .blank   (d_blank),
        .raw     (d_raw),
        .seg_raw ({letter, 24'h0}),
        .valid   (1'b1),                        // данные обновляются каждый такт
        .en      (1'b1),
        .seg     (seg),
        .dig     (dig)
    );

    // -------------------------------------------------------------------------
    // Светодиоды
    // -------------------------------------------------------------------------
    localparam int FLASH_W = $clog2(FLASH_CLKS + 1);

    logic [FLASH_W-1:0] tx_flash, rx_flash;
    logic [3:0]         led_on;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_flash <= '0;
            rx_flash <= '0;
        end else begin
            if (tx_frame)           tx_flash <= FLASH_W'(FLASH_CLKS);
            else if (tx_flash != 0) tx_flash <= tx_flash - 1'b1;

            if (rx_frame)           rx_flash <= FLASH_W'(FLASH_CLKS);
            else if (rx_flash != 0) rx_flash <= rx_flash - 1'b1;
        end
    end

    assign led_on = {err_seen, running, (rx_flash != 0), (tx_flash != 0)};
    assign led    = LED_ACTIVE ? led_on : ~led_on;

endmodule
