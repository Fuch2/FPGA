// =============================================================================
// Конвейер тестовых данных для 4-разрядного индикатора
//
// Выдаёт последовательность кадров для seg7_dynamic (те же поля: hex, dp,
// blank, raw, seg_raw) с импульсом valid на каждый новый кадр.
// Индикатор принимает данные в любом такте, поэтому ready нет.
// Разряд i - это dig[i]; в текстах ниже разряды перечислены от 0 к 3.
//
// Режимы (mode):
//   0 - проверка индикатора, медленно, 12 шагов по кругу:
//       шаги 0..7  - во всех разрядах горит один сегмент: a, b, c, d, e, f, g, dp;
//       шаги 8..11 - «8.» только в разряде 0, 1, 2, 3 (порядок разрядов);
//   1 - десятичный счётчик 0000..9999, быстро;
//   2 - шестнадцатеричный счётчик 0000..FFFF, быстро;
//   3 - таблица (ROM), медленно: 0123, 4567, 89Ab, CdEF, 8.8.8.8., ----,
//       1.2.3.4., пусто.
// Смена режима начинает последовательность с первого кадра и сразу выдаёт его.
// run = 0 - пауза (кадр не меняется), run = 1 - продолжение с того же места.
// =============================================================================
module test_seq #(
    parameter int CLK_FREQ  = 50_000_000,                // частота clk, Гц
    parameter int SLOW_CLKS = CLK_FREQ,                  // шаг режимов 0 и 3 (1 с)
    parameter int FAST_CLKS = CLK_FREQ / 10              // шаг режимов 1 и 2 (0.1 с)
) (
    input  logic        clk,
    input  logic        rst_n,      // асинхронный сброс, активный 0
    input  logic [1:0]  mode,
    input  logic        run,

    output logic [15:0] hex,
    output logic [3:0]  dp,
    output logic [3:0]  blank,
    output logic [3:0]  raw,
    output logic [31:0] seg_raw,
    output logic        valid
);

    localparam int TMR_W = $clog2(SLOW_CLKS > FAST_CLKS ? SLOW_CLKS : FAST_CLKS);

    localparam logic [1:0] M_SEGTEST = 2'd0,
                           M_DEC     = 2'd1,
                           M_HEX     = 2'd2,
                           M_ROM     = 2'd3;

    // -------------------------------------------------------------------------
    // Счётчик шагов и таймер
    // -------------------------------------------------------------------------
    logic [1:0]       mode_r;
    logic [15:0]      cnt;       // номер шага, BCD- или hex-значение счётчика
    logic [TMR_W-1:0] tmr;
    logic             upd;       // 1 - в следующем такте выдать новый кадр

    // следующий шаг текущего режима
    function automatic logic [15:0] next_cnt(input logic [1:0] m, input logic [15:0] c);
        logic [15:0] r;
        logic        carry;
        case (m)
            M_SEGTEST: r = (c == 16'd11) ? 16'd0 : c + 1'b1;
            M_HEX:     r = c + 1'b1;
            M_ROM:     r = (c == 16'd7) ? 16'd0 : c + 1'b1;
            default: begin                        // M_DEC: BCD +1 с переносом
                carry = 1'b1;
                for (int i = 0; i < 4; i++) begin
                    if (carry && c[4*i +: 4] == 4'd9) begin
                        r[4*i +: 4] = 4'd0;
                    end else begin
                        r[4*i +: 4] = c[4*i +: 4] + carry;
                        carry       = 1'b0;
                    end
                end
            end
        endcase
        return r;
    endfunction

    wire              slow     = (mode_r == M_SEGTEST) || (mode_r == M_ROM);
    wire [TMR_W-1:0]  tmr_last = slow ? TMR_W'(SLOW_CLKS - 1) : TMR_W'(FAST_CLKS - 1);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_r <= M_SEGTEST;
            cnt    <= '0;
            tmr    <= '0;
            upd    <= 1'b1;                       // первый кадр сразу после сброса
        end else begin
            upd <= 1'b0;
            if (mode != mode_r) begin             // новый режим - с начала
                mode_r <= mode;
                cnt    <= '0;
                tmr    <= '0;
                upd    <= 1'b1;
            end else if (run) begin
                if (tmr >= tmr_last) begin
                    tmr <= '0;
                    cnt <= next_cnt(mode_r, cnt);
                    upd <= 1'b1;
                end else begin
                    tmr <= tmr + 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Формирование кадра
    // -------------------------------------------------------------------------
    localparam logic [7:0] SEG_G = 8'h40;         // сегмент g = «-»

    // строка из 4 цифр, разряд 0 первым: d0 d1 d2 d3 -> hex
    function automatic logic [15:0] digits(input logic [3:0] d0, d1, d2, d3);
        return {d3, d2, d1, d0};
    endfunction

    logic [15:0] f_hex;
    logic [3:0]  f_dp, f_blank, f_raw;
    logic [31:0] f_seg_raw;

    always_comb begin
        f_hex     = '0;
        f_dp      = '0;
        f_blank   = '0;
        f_raw     = '0;
        f_seg_raw = '0;
        case (mode_r)
            M_SEGTEST: begin
                if (cnt < 16'd8) begin
                    f_raw     = 4'b1111;
                    f_seg_raw = {4{8'(1) << cnt[2:0]}};
                end else begin
                    f_hex   = 16'h8888;
                    f_dp    = 4'b0001 << cnt[1:0];
                    f_blank = ~(4'b0001 << cnt[1:0]);
                end
            end
            M_DEC, M_HEX: f_hex = cnt;
            default: begin                        // M_ROM
                case (cnt[2:0])
                    3'd0: f_hex = digits(4'h0, 4'h1, 4'h2, 4'h3);
                    3'd1: f_hex = digits(4'h4, 4'h5, 4'h6, 4'h7);
                    3'd2: f_hex = digits(4'h8, 4'h9, 4'hA, 4'hB);
                    3'd3: f_hex = digits(4'hC, 4'hD, 4'hE, 4'hF);
                    3'd4: begin f_hex = 16'h8888; f_dp = 4'b1111; end
                    3'd5: begin f_raw = 4'b1111; f_seg_raw = {4{SEG_G}}; end
                    3'd6: begin f_hex = digits(4'h1, 4'h2, 4'h3, 4'h4); f_dp = 4'b1111; end
                    default: f_blank = 4'b1111;
                endcase
            end
        endcase
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hex     <= '0;
            dp      <= '0;
            blank   <= '1;
            raw     <= '0;
            seg_raw <= '0;
            valid   <= 1'b0;
        end else begin
            valid <= upd;
            if (upd) begin
                hex     <= f_hex;
                dp      <= f_dp;
                blank   <= f_blank;
                raw     <= f_raw;
                seg_raw <= f_seg_raw;
            end
        end
    end

endmodule
