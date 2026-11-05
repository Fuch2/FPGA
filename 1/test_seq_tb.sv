// =============================================================================
// Тестбенч для test_seq
//
// Для ускорения шаг медленных режимов = 40 тактов, быстрых = 10 тактов.
// Монитор сравнивает каждый кадр (valid = 1) с эталонной моделью
// и проверяет интервал между кадрами.
// Проверки:
//   1. После сброса сразу выдаётся первый кадр режима 0
//   2. Режим 0: 12 шагов проверки сегментов и разрядов, по кругу
//   3. Режим 1: десятичный счётчик, переход 9999 -> 0000
//   4. Режим 2: шестнадцатеричный счётчик, переход FFFF -> 0000
//   5. Режим 3: таблица из 8 кадров, по кругу
//   6. Пауза (run = 0): кадры не выдаются, после run = 1 - продолжение
//   7. Смена режима посреди шага: новый режим с первого кадра не позже 3 тактов
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module test_seq_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  SLOW     = 40;
    localparam int  FAST     = 10;

    logic        clk   = 1'b0;
    logic        rst_n = 1'b0;
    logic [1:0]  mode  = 2'd0;
    logic        run   = 1'b1;

    logic [15:0] hex;
    logic [3:0]  dp, blank, raw;
    logic [31:0] seg_raw;
    logic        valid;

    always #(CLK_NS / 2) clk = ~clk;

    test_seq #(.CLK_FREQ(CLK_FREQ), .SLOW_CLKS(SLOW), .FAST_CLKS(FAST)) dut (.*);

    // -------------------------------------------------------------------------
    // Эталонная модель
    // -------------------------------------------------------------------------
    typedef struct packed {
        logic [15:0] hex;
        logic [3:0]  dp, blank, raw;
        logic [31:0] seg_raw;
    } frame_t;

    function automatic frame_t model(input int m, input int n);
        frame_t f = '0;
        case (m)
            0: begin
                n = n % 12;
                if (n < 8) begin
                    f.raw     = 4'hF;
                    f.seg_raw = {4{8'(1 << n)}};
                end else begin
                    f.hex   = 16'h8888;
                    f.dp    = 4'(1 << (n - 8));
                    f.blank = ~f.dp;
                end
            end
            1: begin
                n = n % 10000;
                f.hex = {4'(n / 1000), 4'(n / 100 % 10), 4'(n / 10 % 10), 4'(n % 10)};
            end
            2: f.hex = 16'(n);
            default: begin
                case (n % 8)
                    0: f.hex = 16'h3210;
                    1: f.hex = 16'h7654;
                    2: f.hex = 16'hBA98;
                    3: f.hex = 16'hFEDC;
                    4: begin f.hex = 16'h8888; f.dp = 4'hF; end
                    5: begin f.raw = 4'hF; f.seg_raw = 32'h4040_4040; end
                    6: begin f.hex = 16'h4321; f.dp = 4'hF; end
                    7: f.blank = 4'hF;
                endcase
            end
        endcase
        return f;
    endfunction

    // -------------------------------------------------------------------------
    // Монитор
    // -------------------------------------------------------------------------
    int  errors     = 0;
    int  cycle      = 0;
    int  exp_mode   = 0;
    int  exp_idx    = 0;
    int  frames     = 0;
    int  last_valid = -1;    // такт последнего кадра; -1 - интервал не проверять
    int  change_at  = 0;     // такт смены режима
    bit  wait_first = 1'b1;  // ждём первый кадр после сброса или смены режима

    task automatic fail(input string msg);
        errors++;
        if (errors <= 30) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    always @(posedge clk) begin
        frame_t got, exp;
        cycle++;
        #1;
        if (rst_n && wait_first && cycle - change_at > 3) begin
            fail($sformatf("no first frame of mode %0d within 3 clk", exp_mode));
            wait_first = 1'b0;
        end
        if (valid) begin
            got = '{hex, dp, blank, raw, seg_raw};
            exp = model(exp_mode, exp_idx);
            if (got !== exp)
                fail($sformatf("mode %0d frame %0d: got %h/%b/%b/%b/%h, expected %h/%b/%b/%b/%h",
                               exp_mode, exp_idx, got.hex, got.dp, got.blank, got.raw, got.seg_raw,
                               exp.hex, exp.dp, exp.blank, exp.raw, exp.seg_raw));
            if (!run && !wait_first) fail("frame during pause");
            if (last_valid >= 0) begin
                automatic int per = (exp_mode == 0 || exp_mode == 3) ? SLOW : FAST;
                if (cycle - last_valid != per)
                    fail($sformatf("mode %0d: interval %0d clk, expected %0d", exp_mode, cycle - last_valid, per));
            end
            last_valid = cycle;
            wait_first = 1'b0;
            exp_idx++;
            frames++;
        end
    end

    // -------------------------------------------------------------------------
    // Управление
    // -------------------------------------------------------------------------
    task automatic set_mode(input int m);
        @(negedge clk);
        mode       = 2'(m);
        exp_mode   = m;
        exp_idx    = 0;
        last_valid = -1;
        change_at  = cycle;
        wait_first = 1'b1;
    endtask

    task automatic wait_frames(input int n);
        repeat (n) @(posedge clk iff valid);
    endtask

    initial begin
        $display("test_seq_tb: SLOW = %0d clk, FAST = %0d clk", SLOW, FAST);
        repeat (3) @(posedge clk);
        @(negedge clk) begin
            rst_n     = 1'b1;
            change_at = cycle;
        end

        $display("[%0t] 1-2. Mode 0 after reset (segment / digit test), 30 frames", $time);
        wait_frames(30);

        $display("[%0t] 3. Mode 1: decimal counter, 10003 frames (wrap 9999 -> 0000)", $time);
        set_mode(1);
        wait_frames(10003);
        if (hex !== 16'h0002) fail($sformatf("decimal counter after wrap = %h, expected 0002", hex));

        $display("[%0t] 4. Mode 2: hex counter, 65539 frames (wrap FFFF -> 0000)", $time);
        set_mode(2);
        wait_frames(65539);
        if (hex !== 16'h0002) fail($sformatf("hex counter after wrap = %h, expected 0002", hex));

        $display("[%0t] 5. Mode 3: ROM, 20 frames", $time);
        set_mode(3);
        wait_frames(20);

        $display("[%0t] 6. Pause and resume in mode 1", $time);
        set_mode(1);
        wait_frames(5);
        @(negedge clk) run = 1'b0;
        repeat (20 * FAST) @(negedge clk);
        run        = 1'b1;
        last_valid = -1;                 // интервал вокруг паузы не проверяем
        wait_frames(5);
        if (hex !== 16'h0009) fail($sformatf("after pause counter = %h, expected 0009", hex));

        $display("[%0t] 7. Mode change in the middle of a step", $time);
        set_mode(3);
        wait_frames(1);
        repeat (SLOW / 2) @(negedge clk);
        set_mode(2);
        wait_frames(3);
        repeat (FAST / 2) @(negedge clk);
        set_mode(0);
        wait_frames(13);

        repeat (5) @(negedge clk);
        $display("------------------------------------------------------------");
        $display("frames: %0d, ERRORS: %0d", frames, errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #50ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
