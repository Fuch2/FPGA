// =============================================================================
// Индикация обмена по UART: счётчики, светодиоды, семисегментный индикатор
//
// Входы-события - импульсы на 1 такт:
//   rx_byte (+ rx_data)  - принят байт;          tx_byte - передан байт;
//   rx_pkt               - принят верный пакет;  tx_pkt  - передан пакет;
//   garbage              - принят байт, который не команда и не пакет;
//   err_*                - ошибки: кадра UART, контрольной суммы, обрыва пакета,
//                          переполнения (пакет некуда положить).
// Уровень running - автомат команд в состоянии «Старт».
// Управление (импульсы): clear - обнулить счётчики и погасить led[3];
//   next_mode - следующий режим индикатора.
//
// Режимы индикатора (dig[3] - буква, dig[2..0] - число):
//   0  «b XX» - последний принятый байт (hex);
//   1  «rNNN» - число принятых верных пакетов;
//   2  «tNNN» - число переданных пакетов;
//   3  «ENNN» - число ошибок (несколько ошибок в одном такте считаются одной);
//   4  «uNNN» - число байт мусора.
// Счётчики десятичные (BCD), 0..999, после 999 идёт 000.
//
// Светодиоды:
//   led[0] - вспышка при приёме байта;   led[1] - вспышка при передаче байта;
//   led[2] - running (между Старт и Стоп); led[3] - была ошибка (до clear).
// =============================================================================
module uart_ind #(
    parameter int CLK_FREQ   = 50_000_000,      // частота clk, Гц
    parameter int DIGIT_HZ   = 1_000,           // смена разрядов индикатора, Гц
    parameter int BLANK_CLKS = 100,             // гашение между разрядами, тактов
    parameter int FLASH_CLKS = CLK_FREQ / 20,   // длительность вспышки светодиода (50 мс)
    parameter bit SEG_ACTIVE = 1'b0,
    parameter bit DIG_ACTIVE = 1'b0,
    parameter bit LED_ACTIVE = 1'b0
) (
    input  logic       clk,
    input  logic       rst_n,       // асинхронный сброс, активный 0

    input  logic       rx_byte,
    input  logic [7:0] rx_data,
    input  logic       tx_byte,
    input  logic       rx_pkt,
    input  logic       tx_pkt,
    input  logic       garbage,
    input  logic       err_frame,
    input  logic       err_chk,
    input  logic       err_timeout,
    input  logic       err_ovf,
    input  logic       running,

    input  logic       clear,
    input  logic       next_mode,

    output logic [7:0] seg,
    output logic [3:0] dig,
    output logic [3:0] led
);

    localparam int N_MODES = 5;

    typedef enum logic [2:0] {
        M_BYTE    = 3'd0,
        M_RX_PKT  = 3'd1,
        M_TX_PKT  = 3'd2,
        M_ERRORS  = 3'd3,
        M_GARBAGE = 3'd4
    } mode_t;

    // буквы для разряда dig[3], порядок сегментов {dp, g, f, e, d, c, b, a}
    localparam logic [7:0] LET_B = 8'h7C,   // b
                           LET_R = 8'h50,   // r
                           LET_T = 8'h78,   // t
                           LET_E = 8'h79,   // E
                           LET_U = 8'h1C;   // u

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
    // Счётчики и последний байт
    // -------------------------------------------------------------------------
    logic [7:0]  last_byte;
    logic [11:0] cnt_rx_pkt, cnt_tx_pkt, cnt_err, cnt_garbage;
    logic        err_seen;
    mode_t       mode;

    wire any_err = err_frame || err_chk || err_timeout || err_ovf;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_byte   <= '0;
            cnt_rx_pkt  <= '0;
            cnt_tx_pkt  <= '0;
            cnt_err     <= '0;
            cnt_garbage <= '0;
            err_seen    <= 1'b0;
            mode        <= M_BYTE;
        end else begin
            if (next_mode)
                mode <= (mode == N_MODES - 1) ? M_BYTE : mode_t'(mode + 1'b1);

            if (clear) begin
                last_byte   <= '0;
                cnt_rx_pkt  <= '0;
                cnt_tx_pkt  <= '0;
                cnt_err     <= '0;
                cnt_garbage <= '0;
                err_seen    <= 1'b0;
            end else begin
                if (rx_byte) last_byte   <= rx_data;
                if (rx_pkt)  cnt_rx_pkt  <= bcd_inc(cnt_rx_pkt);
                if (tx_pkt)  cnt_tx_pkt  <= bcd_inc(cnt_tx_pkt);
                if (garbage) cnt_garbage <= bcd_inc(cnt_garbage);
                if (any_err) begin
                    cnt_err  <= bcd_inc(cnt_err);
                    err_seen <= 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Индикатор
    // -------------------------------------------------------------------------
    logic [7:0]  letter;
    logic [11:0] number;
    logic [3:0]  d_blank;

    always_comb begin
        d_blank = 4'b0000;
        case (mode)
            M_BYTE: begin
                letter  = LET_B;
                number  = {4'h0, last_byte};
                d_blank = 4'b0100;              // dig[2] погашен: «b XX»
            end
            M_RX_PKT:  begin letter = LET_R; number = cnt_rx_pkt;  end
            M_TX_PKT:  begin letter = LET_T; number = cnt_tx_pkt;  end
            M_ERRORS:  begin letter = LET_E; number = cnt_err;     end
            default:   begin letter = LET_U; number = cnt_garbage; end
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
        .hex     ({4'h0, number}),
        .dp      (4'b0000),
        .blank   (d_blank),
        .raw     (4'b1000),                     // dig[3] - буква, без дешифратора
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

    logic [FLASH_W-1:0] rx_flash, tx_flash;
    logic [3:0]         led_on;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_flash <= '0;
            tx_flash <= '0;
        end else begin
            if (rx_byte)            rx_flash <= FLASH_W'(FLASH_CLKS);
            else if (rx_flash != 0) rx_flash <= rx_flash - 1'b1;

            if (tx_byte)            tx_flash <= FLASH_W'(FLASH_CLKS);
            else if (tx_flash != 0) tx_flash <= tx_flash - 1'b1;
        end
    end

    assign led_on = {err_seen, running, (tx_flash != 0), (rx_flash != 0)};
    assign led    = LED_ACTIVE ? led_on : ~led_on;

endmodule
