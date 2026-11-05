// =============================================================================
// Тестбенч для IND_top (проверка всего тракта индикации)
//
// Параметры уменьшены: слот разряда 100 тактов (гашение 10), период ШИМ
// 255 тактов, антидребезг 20 тактов, шаги test_seq 400 / 100 тактов.
// Проверки:
//   1. После сброса: режим 0, пауза выключена, максимальная яркость
//   2. KEY3 переключает режимы 0 -> 1 -> 2 -> 3 -> 0 (светодиоды led[2:1])
//   3. KEY1 / KEY2 меняют яркость по ступеням с упором в 0 и 7:
//      доля времени, когда горит разряд, = 0.9 * duty / 255 (с допуском)
//   4. Показанные на индикаторе цифры совпадают с данными test_seq
//   5. KEY4 - пауза (led[3], кадр не меняется) и продолжение
//   6. led[0] мигает
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module IND_top_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  SLOT     = 100;
    localparam int  BLANK    = 10;
    localparam int  DEB      = 20;
    localparam int  SLOW     = 400;
    localparam int  FAST     = 100;
    localparam int  HB       = 1000;
    localparam int  WINDOW   = 5100 * 4;     // кратно периодам ШИМ (255) и слота (100)

    logic       clk   = 1'b0;
    logic       rst_n = 1'b0;
    logic [3:0] key   = 4'hF;
    logic [7:0] seg;
    logic [3:0] dig;
    logic [3:0] led;

    always #(CLK_NS / 2) clk = ~clk;

    IND_top #(
        .CLK_FREQ   (CLK_FREQ),
        .DIGIT_HZ   (CLK_FREQ / SLOT),
        .BLANK_CLKS (BLANK),
        .PWM_FREQ   (CLK_FREQ / 255),       // делитель ШИМ = 1
        .DEB_CLKS   (DEB),
        .SLOW_CLKS  (SLOW),
        .FAST_CLKS  (FAST),
        .HB_CLKS    (HB)
    ) dut (.*);

    // -------------------------------------------------------------------------
    // Общее
    // -------------------------------------------------------------------------
    int errors = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 30) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    // выходы в активной 1
    wire [7:0] seg_on = ~seg;
    wire [3:0] dig_on = ~dig;
    wire [3:0] led_on = ~led;

    localparam logic [6:0] SEG_REF [16] = '{
        7'h3F, 7'h06, 7'h5B, 7'h4F, 7'h66, 7'h6D, 7'h7D, 7'h07,
        7'h7F, 7'h6F, 7'h77, 7'h7C, 7'h39, 7'h5E, 7'h79, 7'h71
    };

    // нажать кнопку k (0..3) и отпустить
    task automatic press(input int k);
        @(negedge clk) key[k] = 1'b0;
        repeat (3 * DEB) @(negedge clk);
        key[k] = 1'b1;
        repeat (3 * DEB) @(negedge clk);
    endtask

    // доля тактов, когда горит хоть один разряд
    task automatic measure(output real frac);
        int lit = 0;
        repeat (WINDOW) begin
            @(posedge clk); #1;
            if (dig_on != 0) lit++;
        end
        frac = real'(lit) / WINDOW;
    endtask

    task automatic check_level(input int lvl);
        real f, e;
        repeat (2 * 255) @(negedge clk);          // новая скважность - со следующего периода
        measure(f);
        e = 0.9 * ((2 << lvl) - 1) / 255.0;
        if (f < e * 0.9 - 0.002 || f > e * 1.1 + 0.002)
            fail($sformatf("level %0d: lit fraction %f, expected %f", lvl, f, e));
        else
            $display("    level %0d: duty %3d/255, lit fraction %f (expected %f)", lvl, (2 << lvl) - 1, f, e);
    endtask

    task automatic check_mode(input int m);
        if (led_on[2:1] !== 2'(m))  fail($sformatf("led[2:1] = %b, expected mode %0d", led_on[2:1], m));
        if (dut.u_seq.mode !== 2'(m)) fail($sformatf("test_seq mode = %0d, expected %0d", dut.u_seq.mode, m));
    endtask

    // за один кадр индикации каждый разряд должен показать цифру из test_seq
    task automatic check_digits();
        logic [15:0] h;
        bit [3:0]    seen;
        h    = dut.u_seq.hex;
        seen = '0;
        repeat (8 * SLOT) begin
            @(posedge clk); #1;
            if (dig_on != 0) begin
                automatic int i = $clog2(dig_on);
                seen[i] = 1'b1;
                if (seg_on[6:0] !== SEG_REF[h[4*i +: 4]])
                    fail($sformatf("digit %0d shows %b, expected %h (%b)", i, seg_on[6:0], h[4*i +: 4], SEG_REF[h[4*i +: 4]]));
            end
        end
        if (seen != 4'hF) fail($sformatf("not all digits were on: %b", seen));
        if (dut.u_seq.hex !== h) fail("data changed during check_digits (pause expected)");
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        real f;
        logic [15:0] h0;
        int hb_toggles;

        repeat (5) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        repeat (10) @(negedge clk);

        $display("[%0t] 1. After reset", $time);
        check_mode(0);
        if (led_on[3] !== 1'b0) fail("pause LED is on after reset");

        $display("[%0t] 2. KEY3: modes 1, 2, 3, 0, 1", $time);
        for (int m = 1; m <= 5; m++) begin
            press(2);
            check_mode(m % 4);
        end

        $display("[%0t] 3. Brightness (mode 1, all digits on)", $time);
        check_level(7);
        for (int l = 6; l >= 0; l--) begin
            press(0);
            check_level(l);
        end
        press(0);                       // упор снизу
        check_level(0);
        for (int l = 1; l <= 7; l++) begin
            press(1);
            check_level(l);
        end
        press(1);                       // упор сверху
        check_level(7);

        $display("[%0t] 4-5. Pause, displayed digits, resume", $time);
        press(3);
        if (led_on[3] !== 1'b1) fail("pause LED is off during pause");
        h0 = dut.u_seq.hex;
        check_digits();
        repeat (5 * FAST) @(negedge clk);
        if (dut.u_seq.hex !== h0) fail("counter runs during pause");
        press(3);
        if (led_on[3] !== 1'b0) fail("pause LED is on after resume");
        repeat (5 * FAST) @(negedge clk);
        if (dut.u_seq.hex === h0) fail("counter does not run after resume");

        $display("[%0t] 6. Heartbeat LED", $time);
        hb_toggles = 0;
        repeat (5 * HB) begin
            automatic logic prev = led_on[0];
            @(negedge clk);
            if (led_on[0] != prev) hb_toggles++;
        end
        if (hb_toggles < 4 || hb_toggles > 6) fail($sformatf("led[0] toggled %0d times in 5 half-periods", hb_toggles));

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d", errors);
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
