// =============================================================================
// Индикация обмена по I2C (плата B)
//
// Входы-события - импульсы на 1 такт:
//   ev_start, ev_ack, ev_nack - события шины от i2c_master (START; ведомый
//                               подтвердил / не подтвердил адрес или байт);
//   txn_p, nack_p, err_p      - от i2c_res_parser: транзакция закончена, NACK на
//                               адрес, ошибка (NACK данных, timeout, arb или
//                               неожиданный NACK адреса).
// Данные от i2c_res_parser: info_word (последние два принятых байта), info_addr
// (адрес последней транзакции), info_status (её итог), список найденных
// (found_cnt, found_addr по номеру found_idx).
// Управление: run (импульс) - запущен сценарий, run_scan = 1 - это сканирование
// (покажем список найденных, иначе принятые данные); script_changed (импульс) -
// выбран другой сценарий (покажем его номер, он же на входе script);
// clear - обнулить счётчики и погасить led[3]; next_mode - следующий режим.
//
// Режимы индикатора (dig[3] - буква, dig[2..0] - число; режим 0 - 4 цифры):
//   0  «XXXX»  последние два принятых байта в hex (последний байт - две младшие цифры)
//   1  «A XX»  адрес последней транзакции (hex)
//   2  «S XX»  итог последней транзакции: бит 7 чтение, 3 arb, 2 timeout,
//              1 NACK данных, 0 NACK адреса (S выглядит как 5)
//   3  «tNNN»  транзакций закончено
//   4  «nNNN»  NACK на адрес
//   5  «ENNN»  ошибок
//   6  «Fi.AA» найденные ведомые: i - номер (с 1), AA - адрес hex; список листается
//              по кругу раз в SCROLL_CLKS тактов; «F0.00» - никого нет
//   7  «P   N»  номер выбранного сценария
// Счётчики десятичные (BCD), 0..999, после 999 идёт 000.
//
// Светодиоды:  led[0] - вспышка при START;  led[1] - вспышка при ACK ведомого;
//              led[2] - вспышка при NACK ведомого;  led[3] - была ошибка (до clear).
// =============================================================================
module i2c_ind #(
    parameter int CLK_FREQ    = 50_000_000,     // частота clk, Гц
    parameter int DIGIT_HZ    = 1_000,          // смена разрядов индикатора, Гц
    parameter int BLANK_CLKS  = 100,            // гашение между разрядами, тактов
    parameter int FLASH_CLKS  = CLK_FREQ / 20,  // длительность вспышки светодиода (50 мс)
    parameter int SCROLL_CLKS = CLK_FREQ,       // время показа одного найденного адреса (1 с)
    parameter int N_FOUND     = 8,
    parameter bit SEG_ACTIVE  = 1'b0,
    parameter bit DIG_ACTIVE  = 1'b0,
    parameter bit LED_ACTIVE  = 1'b0
) (
    input  logic                       clk,
    input  logic                       rst_n,       // асинхронный сброс, активный 0

    input  logic                       ev_start,
    input  logic                       ev_ack,
    input  logic                       ev_nack,
    input  logic                       txn_p,
    input  logic                       nack_p,
    input  logic                       err_p,

    input  logic [15:0]                info_word,
    input  logic [6:0]                 info_addr,
    input  logic [7:0]                 info_status,
    input  logic [$clog2(N_FOUND):0]   found_cnt,
    input  logic [6:0]                 found_addr,
    output logic [$clog2(N_FOUND)-1:0] found_idx,

    input  logic [2:0]                 script,
    input  logic                       run,
    input  logic                       run_scan,
    input  logic                       script_changed,
    input  logic                       clear,
    input  logic                       next_mode,

    output logic [7:0]                 seg,
    output logic [3:0]                 dig,
    output logic [3:0]                 led
);

    localparam int N_MODES = 8;
    localparam int IDX_W   = $clog2(N_FOUND);

    typedef enum logic [2:0] {
        M_DATA   = 3'd0,
        M_ADDR   = 3'd1,
        M_STATUS = 3'd2,
        M_TXN    = 3'd3,
        M_NACK   = 3'd4,
        M_ERR    = 3'd5,
        M_FOUND  = 3'd6,
        M_SCRIPT = 3'd7
    } mode_t;

    // буквы для разряда dig[3], порядок сегментов {dp, g, f, e, d, c, b, a}
    localparam logic [7:0] LET_S = 8'h6D,   // S (как 5)
                           LET_T = 8'h78,   // t
                           LET_N = 8'h54,   // n
                           LET_E = 8'h79,   // E
                           LET_P = 8'h73;   // P

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
    // Счётчики, режим, листание списка найденных
    // -------------------------------------------------------------------------
    logic [11:0]               cnt_txn, cnt_nack, cnt_err;
    logic                      err_seen;
    mode_t                     mode;
    logic [$clog2(SCROLL_CLKS+1)-1:0] scroll_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_txn    <= '0;
            cnt_nack   <= '0;
            cnt_err    <= '0;
            err_seen   <= 1'b0;
            mode       <= M_DATA;
            found_idx  <= '0;
            scroll_cnt <= '0;
        end else begin
            if (next_mode)            mode <= mode_t'(mode + 1'b1);
            if (script_changed)       mode <= M_SCRIPT;
            if (run)                  mode <= run_scan ? M_FOUND : M_DATA;

            if (clear) begin
                cnt_txn  <= '0;
                cnt_nack <= '0;
                cnt_err  <= '0;
                err_seen <= 1'b0;
            end else begin
                if (txn_p)  cnt_txn  <= bcd_inc(cnt_txn);
                if (nack_p) cnt_nack <= bcd_inc(cnt_nack);
                if (err_p) begin
                    cnt_err  <= bcd_inc(cnt_err);
                    err_seen <= 1'b1;
                end
            end

            // листание списка найденных
            if (scroll_cnt == SCROLL_CLKS - 1) begin
                scroll_cnt <= '0;
                found_idx  <= (found_cnt == 0 || found_idx + 1'b1 >= found_cnt) ? '0 : found_idx + 1'b1;
            end else begin
                scroll_cnt <= scroll_cnt + 1'b1;
            end
            if (found_cnt == 0 || found_idx >= found_cnt) found_idx <= '0;
        end
    end

    // -------------------------------------------------------------------------
    // Индикатор
    // -------------------------------------------------------------------------
    logic [15:0] number;
    logic [7:0]  letter;
    logic [3:0]  d_blank;
    logic [3:0]  d_raw;
    logic [3:0]  d_dp;

    always_comb begin
        letter  = '0;
        d_blank = 4'b0000;
        d_raw   = 4'b0000;
        d_dp    = 4'b0000;
        number  = '0;
        case (mode)
            M_DATA:   number = info_word;
            M_ADDR: begin
                number  = {4'hA, 4'h0, 1'b0, info_addr};
                d_blank = 4'b0100;                      // «A XX»
            end
            M_STATUS: begin
                letter  = LET_S;
                number  = {8'h00, info_status};
                d_raw   = 4'b1000;
                d_blank = 4'b0100;
            end
            M_TXN: begin
                letter = LET_T;
                number = {4'h0, cnt_txn};
                d_raw  = 4'b1000;
            end
            M_NACK: begin
                letter = LET_N;
                number = {4'h0, cnt_nack};
                d_raw  = 4'b1000;
            end
            M_ERR: begin
                letter = LET_E;
                number = {4'h0, cnt_err};
                d_raw  = 4'b1000;
            end
            M_FOUND: begin
                number = {4'hF, (found_cnt == 0) ? 4'h0 : 4'(found_idx) + 4'h1, 1'b0, found_addr};
                d_dp   = 4'b0100;                       // «Fi.AA»
            end
            default: begin                              // M_SCRIPT
                letter  = LET_P;
                number  = {12'h000, 1'b0, script};
                d_raw   = 4'b1000;
                d_blank = 4'b0110;                      // «P   N»
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
        .seg_raw ({letter, 24'h0}),
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

    assign led_on = {err_seen, (nack_flash != 0), (ack_flash != 0), (start_flash != 0)};
    assign led    = LED_ACTIVE ? led_on : ~led_on;

endmodule
