// =============================================================================
// Индикация проверки EEPROM (плата B): адрес, данные, указатели кольца и светодиоды
//
// Входы - состояние eeprom_seq и eeprom_store: info_addr, info_data, info_empty (последняя
// операция), cursor, ring_count, ring_wr_ptr, ring_rd_ptr, polls_last, script (выбранный
// сценарий); события-импульсы wr_p, rd_p (начата запись / чтение), err_p (ошибка);
// уровни polling (микросхема занята циклом записи) и busy (сценарий выполняется).
// Управление: next_mode - следующий режим; script_changed (импульс) - показать номер
// сценария; run (импульс) - вернуться к автоматическому режиму; clear - обнулить счётчик
// ошибок и погасить светодиод ERR.
//
// Режимы индикатора (dig[3] - буква, справа три цифры; точка после буквы):
//   0  АВТО    попеременно, каждые AUTO_CLKS тактов: адрес «A.XXX» и данные «XXXX»
//   1  данные  «XXXX» hex: записанное или прочитанное слово; «----», если читать было нечего
//   2  адрес   «A.XXX» hex: адрес ячейки последней операции (10 бит)
//   3  курсор  «c.XXX» hex: адрес, с которого работают WR_BYTE, WR_WORD, RD_*
//   4  голова  «H.XXX» hex: указатель записи кольца (номер записи)
//              (на экранах кольца 4, 5, 6 точка в младшем разряде горит, если кольцо переполнялось)
//   5  хвост   «t.XXX» hex: указатель чтения кольца (номер записи)
//   6  записей «n.NNN» десятичное: сколько записей кольца ещё не прочитано
//   7  опросов «P.NNN» десятичное: сколько опросов ACK заняла последняя запись (цикл
//              записи микросхемы около 120 мкс на опрос)
//   8  ошибок  «E.NNN» десятичное: сколько операций закончилось ошибкой EEPROM
//   9  сценарий «S.   N» номер выбранного сценария (S выглядит как 5); показывается
//              после смены сценария, в цикл next_mode не входит
//
// Светодиоды (на плате нумерация обратная: LED1 = led[3]):
//   led[3] WR   - вспышка при каждой записи;   led[2] RD  - вспышка при каждом чтении;
//   led[1] BUSY - микросхема занята циклом записи (вспышка, пока идёт опрос ACK);
//   led[0] ERR  - была ошибка EEPROM (до clear).
// =============================================================================
module eeprom_ind #(
    parameter int CLK_FREQ    = 50_000_000,     // частота clk, Гц
    parameter int ADDR_W      = 10,
    parameter int COUNT_W     = 10,             // разрядность ring_count (не больше 10)
    parameter int PTR_W       = 9,
    parameter int AUTO_CLKS   = CLK_FREQ,       // время показа одного экрана в режиме АВТО (1 с)
    parameter int DIGIT_HZ    = 1_000,          // смена разрядов индикатора, Гц
    parameter int BLANK_CLKS  = 100,            // гашение между разрядами, тактов
    parameter int FLASH_CLKS  = CLK_FREQ / 20,  // длительность вспышки светодиода (50 мс)
    parameter bit SEG_ACTIVE  = 1'b0,
    parameter bit DIG_ACTIVE  = 1'b0,
    parameter bit LED_ACTIVE  = 1'b0
) (
    input  logic                clk,
    input  logic                rst_n,          // асинхронный сброс, активный 0

    input  logic [ADDR_W-1:0]   info_addr,
    input  logic [15:0]         info_data,
    input  logic                info_empty,
    input  logic [ADDR_W-1:0]   cursor,
    input  logic [COUNT_W-1:0]  ring_count,
    input  logic [PTR_W-1:0]    ring_wr_ptr,
    input  logic [PTR_W-1:0]    ring_rd_ptr,
    input  logic                ring_ovf,
    input  logic [7:0]          polls_last,
    input  logic [3:0]          script,

    input  logic                wr_p,
    input  logic                rd_p,
    input  logic                err_p,
    input  logic                polling,

    input  logic                next_mode,
    input  logic                script_changed,
    input  logic                run,
    input  logic                clear,

    output logic [7:0]          seg,
    output logic [3:0]          dig,
    output logic [3:0]          led
);

    typedef enum logic [3:0] {
        M_AUTO   = 4'd0,
        M_DATA   = 4'd1,
        M_ADDR   = 4'd2,
        M_CURSOR = 4'd3,
        M_HEAD   = 4'd4,
        M_TAIL   = 4'd5,
        M_COUNT  = 4'd6,
        M_POLLS  = 4'd7,
        M_ERR    = 4'd8,
        M_SCRIPT = 4'd9
    } mode_t;

    localparam int N_MODES = 9;                 // режимы 0..8 листаются кнопкой

    // буквы и знаки, порядок сегментов {dp, g, f, e, d, c, b, a}
    localparam logic [7:0] SEG_DP    = 8'h80,   // десятичная точка в seg_raw
                           SEG_MINUS = 8'h40,
                           LET_A     = 8'h77,   // A
                           LET_C     = 8'h58,   // c
                           LET_H     = 8'h76,   // H
                           LET_T     = 8'h78,   // t
                           LET_N     = 8'h54,   // n
                           LET_P     = 8'h73,   // P
                           LET_E     = 8'h79,   // E
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
    // Перевод чисел в десятичные цифры (постоянно, по кругу)
    // -------------------------------------------------------------------------
    logic        cnt_busy, polls_busy;
    logic [15:0] count_bcd;
    logic [11:0] polls_bcd;

    bin2bcd #(.IN_W (10), .DIGITS (4)) u_count (
        .clk (clk), .rst_n (rst_n), .start (!cnt_busy), .bin (10'(ring_count)),
        .busy (cnt_busy), .bcd (count_bcd)
    );

    bin2bcd #(.IN_W (8), .DIGITS (3)) u_polls (
        .clk (clk), .rst_n (rst_n), .start (!polls_busy), .bin (polls_last),
        .busy (polls_busy), .bcd (polls_bcd)
    );

    // -------------------------------------------------------------------------
    // Режим, счётчик ошибок, чередование экранов
    // -------------------------------------------------------------------------
    mode_t                          mode;
    logic [11:0]                    cnt_err;
    logic                           err_seen;
    logic                           auto_data;      // в режиме АВТО сейчас экран данных
    logic [$clog2(AUTO_CLKS+1)-1:0] auto_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode      <= M_AUTO;
            cnt_err   <= '0;
            err_seen  <= 1'b0;
            auto_data <= 1'b0;
            auto_cnt  <= '0;
        end else begin
            if (next_mode)       mode <= (mode >= mode_t'(N_MODES - 1)) ? M_AUTO : mode_t'(mode + 1'b1);
            if (script_changed)  mode <= M_SCRIPT;
            if (run)             mode <= M_AUTO;

            if (clear) begin
                cnt_err  <= '0;
                err_seen <= 1'b0;
            end else if (err_p) begin
                cnt_err  <= bcd_inc(cnt_err);
                err_seen <= 1'b1;
            end

            if (auto_cnt == AUTO_CLKS - 1) begin
                auto_cnt  <= '0;
                auto_data <= !auto_data;
            end else begin
                auto_cnt <= auto_cnt + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Экран
    // -------------------------------------------------------------------------
    mode_t       screen;
    logic [15:0] number;
    logic [3:0]  d_blank, d_raw, d_dp;
    logic [31:0] raw_seg;                           // {dig[3], dig[2], dig[1], dig[0]} по 8 бит

    always_comb begin
        screen  = (mode == M_AUTO) ? (auto_data ? M_DATA : M_ADDR) : mode;
        number  = '0;
        d_blank = 4'b0000;
        d_raw   = 4'b0000;
        d_dp    = 4'b0000;
        raw_seg = '0;
        case (screen)
            M_DATA: begin
                if (info_empty) begin
                    d_raw   = 4'b1111;
                    raw_seg = {4{SEG_MINUS}};
                end else begin
                    number = info_data;
                end
            end
            M_ADDR: begin
                number  = 16'(info_addr);
                d_raw   = 4'b1000;
                raw_seg = {LET_A | SEG_DP, 24'h0};
            end
            M_CURSOR: begin
                number  = 16'(cursor);
                d_raw   = 4'b1000;
                raw_seg = {LET_C | SEG_DP, 24'h0};
            end
            M_HEAD: begin
                number  = 16'(ring_wr_ptr);
                d_raw   = 4'b1000;
                d_dp    = {3'b000, ring_ovf};
                raw_seg = {LET_H | SEG_DP, 24'h0};
            end
            M_TAIL: begin
                number  = 16'(ring_rd_ptr);
                d_raw   = 4'b1000;
                d_dp    = {3'b000, ring_ovf};
                raw_seg = {LET_T | SEG_DP, 24'h0};
            end
            M_COUNT: begin
                number  = {4'h0, count_bcd[11:0]};
                d_raw   = 4'b1000;
                d_dp    = {3'b000, ring_ovf};
                raw_seg = {LET_N | SEG_DP, 24'h0};
            end
            M_POLLS: begin
                number  = {4'h0, polls_bcd};
                d_raw   = 4'b1000;
                raw_seg = {LET_P | SEG_DP, 24'h0};
            end
            M_ERR: begin
                number  = {4'h0, cnt_err};
                d_raw   = 4'b1000;
                raw_seg = {LET_E | SEG_DP, 24'h0};
            end
            default: begin                          // M_SCRIPT: «S.   N»
                number  = {12'h000, script};
                d_raw   = 4'b1000;
                d_blank = 4'b0110;
                raw_seg = {LET_S | SEG_DP, 24'h0};
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
        .valid   (1'b1),                            // данные обновляются каждый такт
        .en      (1'b1),
        .seg     (seg),
        .dig     (dig)
    );

    // -------------------------------------------------------------------------
    // Светодиоды
    // -------------------------------------------------------------------------
    localparam int FLASH_W = $clog2(FLASH_CLKS + 1);

    logic [FLASH_W-1:0] wr_flash, rd_flash, busy_flash;
    logic [3:0]         led_on;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_flash   <= '0;
            rd_flash   <= '0;
            busy_flash <= '0;
        end else begin
            if (wr_p)                    wr_flash <= FLASH_W'(FLASH_CLKS);
            else if (wr_flash != 0)      wr_flash <= wr_flash - 1'b1;

            if (rd_p)                    rd_flash <= FLASH_W'(FLASH_CLKS);
            else if (rd_flash != 0)      rd_flash <= rd_flash - 1'b1;

            if (polling)                 busy_flash <= FLASH_W'(FLASH_CLKS);
            else if (busy_flash != 0)    busy_flash <= busy_flash - 1'b1;
        end
    end

    assign led_on = {(wr_flash != 0), (rd_flash != 0), (busy_flash != 0), err_seen};
    assign led    = LED_ACTIVE ? led_on : ~led_on;

endmodule
