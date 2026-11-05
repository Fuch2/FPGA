// =============================================================================
// Динамическая индикация на N-разрядном семисегментном индикаторе
//
// Разряды зажигаются по очереди: dig[0], dig[1], ... dig[N-1], каждый на время
// слота 1/DIGIT_HZ. В начале каждого слота BLANK_CLKS тактов всё погашено,
// чтобы при переключении разряда не было «призраков» соседней цифры.
//
// Данные (интерфейс valid): hex, dp, blank, raw, seg_raw защёлкиваются
// в такте, где valid = 1, и показываются до следующего valid. Модуль готов
// принять данные в любом такте, поэтому сигнала ready нет.
//   hex[4i+3:4i]       - шестнадцатеричная цифра разряда i (BCD - частный случай);
//   dp[i]              - десятичная точка разряда i;
//   blank[i]           - разряд i погашен;
//   raw[i]             - разряд i показывает seg_raw[8i+7:8i] как есть, без дешифратора.
//
// Порядок сегментов на выходе seg (и в seg_raw): {dp, g, f, e, d, c, b, a},
// то есть seg[0] = a ... seg[6] = g, seg[7] = dp. Активные уровни задаются
// параметрами, внутри модуля 1 = сегмент/разряд горит.
//
// Вход en гасит индикатор целиком (en = 0), на него подаётся ШИМ яркости.
// =============================================================================
module seg7_dynamic #(
    parameter int CLK_FREQ   = 50_000_000,  // частота clk, Гц
    parameter int N          = 4,           // число разрядов
    parameter int DIGIT_HZ   = 1_000,       // частота смены разрядов, Гц (кадр = N / DIGIT_HZ)
    parameter int BLANK_CLKS = 100,         // гашение в начале слота, тактов clk
    parameter bit SEG_ACTIVE = 1'b0,        // уровень, зажигающий сегмент
    parameter bit DIG_ACTIVE = 1'b0         // уровень, включающий разряд
) (
    input  logic           clk,
    input  logic           rst_n,       // асинхронный сброс, активный 0

    input  logic [4*N-1:0] hex,
    input  logic [N-1:0]   dp,
    input  logic [N-1:0]   blank,
    input  logic [N-1:0]   raw,
    input  logic [8*N-1:0] seg_raw,
    input  logic           valid,

    input  logic           en,          // 1 - индикатор включён (ШИМ яркости)

    output logic [7:0]     seg,
    output logic [N-1:0]   dig
);

    localparam int SLOT_CLKS = CLK_FREQ / DIGIT_HZ;
    localparam int SLOT_W    = $clog2(SLOT_CLKS + 1);
    localparam int IDX_W     = (N > 1) ? $clog2(N) : 1;

    // -------------------------------------------------------------------------
    // Дешифратор: шестнадцатеричная цифра -> сегменты {g, f, e, d, c, b, a}
    // -------------------------------------------------------------------------
    function automatic logic [6:0] hex2seg(input logic [3:0] h);
        case (h)
            4'h0: hex2seg = 7'b011_1111;
            4'h1: hex2seg = 7'b000_0110;
            4'h2: hex2seg = 7'b101_1011;
            4'h3: hex2seg = 7'b100_1111;
            4'h4: hex2seg = 7'b110_0110;
            4'h5: hex2seg = 7'b110_1101;
            4'h6: hex2seg = 7'b111_1101;
            4'h7: hex2seg = 7'b000_0111;
            4'h8: hex2seg = 7'b111_1111;
            4'h9: hex2seg = 7'b110_1111;
            4'hA: hex2seg = 7'b111_0111;
            4'hB: hex2seg = 7'b111_1100;   // b
            4'hC: hex2seg = 7'b011_1001;
            4'hD: hex2seg = 7'b101_1110;   // d
            4'hE: hex2seg = 7'b111_1001;
            default: hex2seg = 7'b111_0001; // F
        endcase
    endfunction

    // -------------------------------------------------------------------------
    // Регистры отображаемых данных
    // -------------------------------------------------------------------------
    logic [4*N-1:0] hex_r;
    logic [N-1:0]   dp_r, blank_r, raw_r;
    logic [8*N-1:0] seg_raw_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hex_r     <= '0;
            dp_r      <= '0;
            blank_r   <= '1;               // после сброса индикатор погашен
            raw_r     <= '0;
            seg_raw_r <= '0;
        end else if (valid) begin
            hex_r     <= hex;
            dp_r      <= dp;
            blank_r   <= blank;
            raw_r     <= raw;
            seg_raw_r <= seg_raw;
        end
    end

    // -------------------------------------------------------------------------
    // Развёртка: счётчик слота и номер текущего разряда
    // -------------------------------------------------------------------------
    logic [SLOT_W-1:0] slot_cnt;
    logic [IDX_W-1:0]  idx;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            slot_cnt <= '0;
            idx      <= '0;
        end else if (slot_cnt == SLOT_CLKS - 1) begin
            slot_cnt <= '0;
            idx      <= (idx == N - 1) ? '0 : idx + 1'b1;
        end else begin
            slot_cnt <= slot_cnt + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // Выходы (регистровые, без дребезга комбинационной логики на выводах)
    // -------------------------------------------------------------------------
    logic [7:0] pattern;
    logic       lit;

    always_comb begin
        if (raw_r[idx]) pattern = seg_raw_r[8*idx +: 8];
        else            pattern = {dp_r[idx], hex2seg(hex_r[4*idx +: 4])};
        lit = en && !blank_r[idx] && (slot_cnt >= BLANK_CLKS);
    end

    logic [7:0]   seg_on;
    logic [N-1:0] dig_on;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seg_on <= '0;
            dig_on <= '0;
        end else begin
            seg_on <= lit ? pattern : '0;
            dig_on <= lit ? (N'(1) << idx) : '0;
        end
    end

    assign seg = SEG_ACTIVE ? seg_on : ~seg_on;
    assign dig = DIG_ACTIVE ? dig_on : ~dig_on;

endmodule
