// =============================================================================
// Тестбенч для debounce
//
// Два экземпляра: L - кнопка с активным 0 (как на плате), H - с активным 1,
// на H подаётся инверсный сигнал; их выходы должны совпадать.
// Для ускорения STABLE_CLKS = 50.
// Проверки:
//   1. После сброса: кнопка отпущена, импульсов нет
//   2. Чистое нажатие и отпускание: ровно по одному импульсу,
//      задержка от последнего изменения входа STABLE_CLKS..STABLE_CLKS+3 тактов
//   3. Нажатие и отпускание с дребезгом (случайные импульсы короче STABLE_CLKS)
//   4. Одиночные помехи короче STABLE_CLKS не дают срабатывания
//   5. pressed меняется только вместе с press_p / release_p
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module debounce_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  STABLE   = 50;

    logic clk   = 1'b0;
    logic rst_n = 1'b0;
    logic btn   = 1'b1;                  // вывод кнопки, отпущена = 1

    logic pressed_l, press_l, release_l;
    logic pressed_h, press_h, release_h;

    always #(CLK_NS / 2) clk = ~clk;

    debounce #(.CLK_FREQ(CLK_FREQ), .STABLE_CLKS(STABLE), .ACTIVE(1'b0)) dut_l (
        .clk, .rst_n, .btn(btn),
        .pressed(pressed_l), .press_p(press_l), .release_p(release_l));

    debounce #(.CLK_FREQ(CLK_FREQ), .STABLE_CLKS(STABLE), .ACTIVE(1'b1)) dut_h (
        .clk, .rst_n, .btn(~btn),
        .pressed(pressed_h), .press_p(press_h), .release_p(release_h));

    // -------------------------------------------------------------------------
    // Монитор
    // -------------------------------------------------------------------------
    int  errors      = 0;
    int  cycle       = 0;
    int  last_change = 0;      // такт последнего изменения btn
    int  n_press     = 0;
    int  n_release   = 0;
    bit  prev_pressed = 1'b0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 30) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    always @(btn) last_change = cycle;

    always @(posedge clk) begin
        cycle++;
        #1;
        if ({pressed_l, press_l, release_l} !== {pressed_h, press_h, release_h})
            fail("active-0 and active-1 instances differ");

        if (press_l && release_l) fail("press_p and release_p at once");

        if (press_l || release_l) begin
            automatic int d = cycle - last_change;
            if (d < STABLE || d > STABLE + 3)
                fail($sformatf("%s %0d clk after last input change, expected %0d..%0d",
                               press_l ? "press" : "release", d, STABLE, STABLE + 3));
        end
        if (press_l)   n_press++;
        if (release_l) n_release++;

        if (pressed_l != prev_pressed && !(pressed_l ? press_l : release_l))
            fail("pressed changed without pulse");
        if ((press_l && !pressed_l) || (release_l && pressed_l))
            fail("pulse without pressed change");
        prev_pressed = pressed_l;
    end

    // -------------------------------------------------------------------------
    // Задачи формирования входа (изменения по спаду clk)
    // -------------------------------------------------------------------------
    task automatic hold(input logic level, input int clks);
        @(negedge clk) btn = level;
        repeat (clks - 1) @(negedge clk);
    endtask

    // дребезг: n случайных переключений короче STABLE, затем уровень level
    task automatic bounce_to(input logic level, input int n);
        for (int i = 0; i < n; i++) begin
            hold(level, $urandom_range(1, STABLE - 10));
            hold(~level, $urandom_range(1, STABLE - 10));
        end
        hold(level, 3 * STABLE);
    endtask

    task automatic expect_counts(input int p, input int r, input string what);
        if (n_press != p || n_release != r)
            fail($sformatf("%s: presses %0d, releases %0d, expected %0d, %0d",
                           what, n_press, n_release, p, r));
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        $display("debounce_tb: STABLE_CLKS = %0d", STABLE);
        repeat (5) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;

        $display("[%0t] 1. After reset", $time);
        repeat (3 * STABLE) @(negedge clk);
        if (pressed_l !== 1'b0) fail("pressed after reset");
        expect_counts(0, 0, "after reset");

        $display("[%0t] 2. Clean press and release", $time);
        hold(1'b0, 3 * STABLE);
        expect_counts(1, 0, "clean press");
        if (pressed_l !== 1'b1) fail("not pressed after clean press");
        hold(1'b1, 3 * STABLE);
        expect_counts(1, 1, "clean release");

        $display("[%0t] 3. Bouncing press and release (x20)", $time);
        for (int k = 0; k < 20; k++) begin
            bounce_to(1'b0, $urandom_range(1, 8));
            expect_counts(2 + k, 1 + k, $sformatf("bounce press %0d", k));
            bounce_to(1'b1, $urandom_range(1, 8));
            expect_counts(2 + k, 2 + k, $sformatf("bounce release %0d", k));
        end

        $display("[%0t] 4. Short glitches are ignored", $time);
        for (int len = 1; len < STABLE - 2; len += 7) begin
            hold(1'b0, len);                 // помеха при отпущенной кнопке
            hold(1'b1, 2 * STABLE);
        end
        expect_counts(21, 21, "glitches while released");
        hold(1'b0, 3 * STABLE);              // нажать
        for (int len = 1; len < STABLE - 2; len += 7) begin
            hold(1'b1, len);                 // помеха при нажатой кнопке
            hold(1'b0, 2 * STABLE);
        end
        expect_counts(22, 21, "glitches while pressed");
        hold(1'b1, 3 * STABLE);
        expect_counts(22, 22, "final release");

        $display("------------------------------------------------------------");
        $display("presses: %0d, releases: %0d, ERRORS: %0d", n_press, n_release, errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #10ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
