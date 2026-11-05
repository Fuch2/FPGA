// =============================================================================
// Тестбенч для seg7_dynamic
//
// Два экземпляра с одинаковыми входами: L - активный 0 (как на плате),
// H - активный 1. Для ускорения слот = 100 тактов, гашение = 10 тактов.
// Монитор каждый такт сравнивает выходы с эталонной моделью и проверяет:
//   1. После сброса всё погашено
//   2. Одновременно горит не больше одного разряда, порядок 0, 1, 2, 3, 0, ...
//   3. Сегменты горящего разряда = эталонный дешифратор / seg_raw / точка
//   4. Каждый разряд горит SLOT - BLANK тактов за слот, между разными
//      разрядами пауза не меньше BLANK тактов
//   5. Все 16 цифр в каждом разряде, точки, гашение разрядов, режим raw
//   6. Без valid изменение входов не влияет на индикацию
//   7. en = 0 гасит всё (случайный «ШИМ» на en)
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module seg7_dynamic_tb;

    localparam int  CLK_FREQ   = 50_000_000;
    localparam real CLK_NS     = 1.0e9 / CLK_FREQ;
    localparam int  N          = 4;
    localparam int  DIGIT_HZ   = 500_000;
    localparam int  SLOT       = CLK_FREQ / DIGIT_HZ;     // 100 тактов
    localparam int  BLANK      = 10;
    localparam int  FRAME      = N * SLOT;

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic           clk     = 1'b0;
    logic           rst_n   = 1'b0;
    logic [4*N-1:0] hex     = '0;
    logic [N-1:0]   dp      = '0;
    logic [N-1:0]   blank   = '0;
    logic [N-1:0]   raw     = '0;
    logic [8*N-1:0] seg_raw = '0;
    logic           valid   = 1'b0;
    logic           en      = 1'b1;

    logic [7:0]     seg_l, seg_h;
    logic [N-1:0]   dig_l, dig_h;

    always #(CLK_NS / 2) clk = ~clk;

    seg7_dynamic #(
        .CLK_FREQ(CLK_FREQ), .N(N), .DIGIT_HZ(DIGIT_HZ), .BLANK_CLKS(BLANK),
        .SEG_ACTIVE(1'b0), .DIG_ACTIVE(1'b0)
    ) dut_l (.clk, .rst_n, .hex, .dp, .blank, .raw, .seg_raw, .valid, .en,
             .seg(seg_l), .dig(dig_l));

    seg7_dynamic #(
        .CLK_FREQ(CLK_FREQ), .N(N), .DIGIT_HZ(DIGIT_HZ), .BLANK_CLKS(BLANK),
        .SEG_ACTIVE(1'b1), .DIG_ACTIVE(1'b1)
    ) dut_h (.clk, .rst_n, .hex, .dp, .blank, .raw, .seg_raw, .valid, .en,
             .seg(seg_h), .dig(dig_h));

    // -------------------------------------------------------------------------
    // Эталон
    // -------------------------------------------------------------------------
    // сегменты {g..a} для цифр 0..F (по таблице стандартного начертания)
    localparam logic [6:0] SEG_REF [16] = '{
        7'h3F, 7'h06, 7'h5B, 7'h4F, 7'h66, 7'h6D, 7'h7D, 7'h07,
        7'h7F, 7'h6F, 7'h77, 7'h7C, 7'h39, 7'h5E, 7'h79, 7'h71
    };

    typedef struct {
        logic [4*N-1:0] hex;
        logic [N-1:0]   dp, blank, raw;
        logic [8*N-1:0] seg_raw;
    } disp_t;

    disp_t m_lat;        // что DUT защёлкнул
    disp_t m_out;        // по чему построены текущие выходы (на такт позже)

    function automatic logic [7:0] expected_seg(disp_t m, int i);
        if (m.raw[i]) return m.seg_raw[8*i +: 8];
        return {m.dp[i], SEG_REF[m.hex[4*i +: 4]]};
    endfunction

    int errors = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 30) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    // -------------------------------------------------------------------------
    // Монитор
    // -------------------------------------------------------------------------
    int  lit_cnt [N];    // сколько тактов горел каждый разряд (с последнего обнуления)
    int  run_len;        // длина текущего горения разряда
    int  dark_len;       // длина текущей паузы
    int  last_dig;       // последний горевший разряд
    bit  checking = 1'b0;

    initial begin
        m_lat    = '{hex: '0, dp: '0, blank: '1, raw: '0, seg_raw: '0};
        m_out    = m_lat;
        last_dig = -1;
        run_len  = 0;
        dark_len = 0;
        foreach (lit_cnt[i]) lit_cnt[i] = 0;
    end

    always @(posedge clk) begin : monitor
        logic       en_s;
        logic [7:0] sl, sh;
        logic [N-1:0] dl, dh;
        int         cur;

        // модель: выходы после этого фронта построены по данным, защёлкнутым раньше
        en_s  = en;
        m_out = m_lat;
        if (valid && rst_n) m_lat = '{hex: hex, dp: dp, blank: blank, raw: raw, seg_raw: seg_raw};

        #1;
        if (!checking) begin
            // после сброса и до начала проверок всё должно быть погашено
            if (dig_l !== '1 || seg_l !== '1 || dig_h !== '0 || seg_h !== '0)
                fail("display is not dark after reset");
        end else begin
            sl = ~seg_l;  dl = ~dig_l;      // L-экземпляр -> активная 1
            sh =  seg_h;  dh =  dig_h;

            if (sl !== sh || dl !== dh)
                fail($sformatf("L and H differ: L seg %h dig %b, H seg %h dig %b", sl, dl, sh, dh));
            if (!$onehot0(dl))
                fail($sformatf("several digits on: %b", dl));

            if (dl == '0) begin
                if (sl != '0) fail($sformatf("segments %h on while no digit is on", sl));
                dark_len++;
                run_len = 0;
            end else begin
                cur = $clog2(dl);
                if (!en_s)            fail("digit on while en = 0");
                if (m_out.blank[cur]) fail($sformatf("blanked digit %0d is on", cur));
                if (sl !== expected_seg(m_out, cur))
                    fail($sformatf("digit %0d: seg %b, expected %b", cur, sl, expected_seg(m_out, cur)));
                if (cur != last_dig && last_dig >= 0) begin
                    if (dark_len < BLANK)
                        fail($sformatf("only %0d dark clk between digit %0d and %0d", dark_len, last_dig, cur));
                    // без погашенных разрядов следующий горящий - соседний
                    if (m_out.blank == '0 && cur != (last_dig + 1) % N)
                        fail($sformatf("digit order: %0d after %0d", cur, last_dig));
                end
                lit_cnt[cur]++;
                run_len++;
                if (run_len > SLOT - BLANK)
                    fail($sformatf("digit %0d is on longer than %0d clk", cur, SLOT - BLANK));
                last_dig = cur;
                dark_len = 0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Вспомогательные задачи
    // -------------------------------------------------------------------------
    task automatic load(input logic [4*N-1:0] h, input logic [N-1:0] d,
                        input logic [N-1:0] b, input logic [N-1:0] r,
                        input logic [8*N-1:0] s);
        @(negedge clk);
        hex = h; dp = d; blank = b; raw = r; seg_raw = s;
        valid = 1'b1;
        @(negedge clk);
        valid = 1'b0;
    endtask

    // ждать начала слота разряда 0 (по внутреннему счётчику DUT)
    task automatic sync_frame();
        @(posedge clk iff (dut_l.idx == 0 && dut_l.slot_cnt == 0));
        @(negedge clk);
    endtask

    // прогнать k кадров с en = 1 и проверить время горения каждого разряда
    task automatic check_frames(input int k, input string what);
        sync_frame();
        foreach (lit_cnt[i]) lit_cnt[i] = 0;
        repeat (k * FRAME) @(negedge clk);
        for (int i = 0; i < N; i++) begin
            automatic int exp = m_lat.blank[i] ? 0 : k * (SLOT - BLANK);
            if (lit_cnt[i] != exp)
                fail($sformatf("%s: digit %0d on %0d clk, expected %0d", what, i, lit_cnt[i], exp));
        end
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        $display("seg7_dynamic_tb: N = %0d, slot = %0d clk, blank = %0d clk", N, SLOT, BLANK);

        repeat (5) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;

        $display("[%0t] 1. Reset: display is dark until data is loaded", $time);
        repeat (2 * FRAME) @(negedge clk);
        checking = 1'b1;

        $display("[%0t] 2. Digits 0123 without dots", $time);
        load(16'h3210, 4'b0000, 4'b0000, 4'b0000, '0);
        check_frames(3, "0123");

        $display("[%0t] 3. All 16 hex digits in every position, dots", $time);
        for (int v = 0; v < 16; v++) begin
            automatic logic [15:0] h;
            for (int i = 0; i < N; i++) h[4*i +: 4] = 4'(v + i);
            load(h, 4'(v), 4'b0000, 4'b0000, '0);
            check_frames(1, $sformatf("hex %0d", v));
        end

        $display("[%0t] 4. Blanked digits", $time);
        load(16'h8888, 4'b1111, 4'b0101, 4'b0000, '0);
        check_frames(2, "blank 0101");
        load(16'h8888, 4'b1111, 4'b1111, 4'b0000, '0);
        check_frames(1, "blank 1111");

        $display("[%0t] 5. Raw segments", $time);
        load(16'h1234, 4'b0000, 4'b0000, 4'b1010, {8'h80, 8'h40, 8'h01, 8'hFF});
        check_frames(2, "raw 1010");
        for (int k = 0; k < 8; k++) begin
            load('0, '0, '0, 4'b1111, {4{8'(1 << k)}});
            check_frames(1, $sformatf("raw segment %0d", k));
        end

        $display("[%0t] 6. Inputs change without valid", $time);
        load(16'h5A5A, 4'b0110, 4'b0000, 4'b0000, '0);
        @(negedge clk);
        hex = 16'hFFFF; dp = 4'b1111; blank = 4'b1111; raw = 4'b1111; seg_raw = '1;
        check_frames(2, "no valid");

        $display("[%0t] 7. Random en (brightness PWM)", $time);
        hex = 16'h0F0F; dp = 4'b1001; blank = 4'b0000; raw = 4'b0000;
        @(negedge clk) valid = 1'b1;
        @(negedge clk) valid = 1'b0;
        repeat (6 * FRAME) begin
            @(negedge clk);
            en = ($urandom_range(0, 3) != 0);
        end
        en = 1'b1;
        check_frames(1, "after random en");

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d", errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #5ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
