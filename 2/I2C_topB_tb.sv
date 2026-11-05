// =============================================================================
// Тестбенч для I2C_topB (плата B целиком): кнопки -> сценарий -> I2C-мастер -> шина с
// моделями «датчика» (0x48) и «EEPROM» (0x50..0x53) -> парсер -> светодиоды и индикатор
//
// Параметры уменьшены: антидребезг 20 тактов, слот индикатора 20 тактов, вспышка 30 тактов,
// листание найденных адресов 20000 тактов. Шина настоящая: 100 кГц.
// Картинку индикатора тестбенч «читает» с выводов seg / dig (как глаз), светодиоды
// отслеживает по ходу сценария. Номер режима для перехода берётся из DUT, но то, что
// показано, сравнивается с независимо вычисленной строкой.
// Проверки:
//   1. После сброса: светодиоды погашены, индикатор «0000»
//   2. KEY2: сценарий 1, индикатор «P  1»
//   3. KEY1 DT_TEMP: «1960»; были START и ACK, NACK не было; режимы «A 48», «5 80» (S), «t002», «n000», «E000»
//   4. KEY2, KEY1 DT_TOS: «5000»
//   5. SCAN (сценарий 0): индикатор сам переходит в «Fi.AA», листает «F1.48» ... «F5.53»;
//      счётчики t112, n107, E000; были START, ACK и NACK
//   6. ABSENT (сценарий 5): n108, ошибок нет, led[2] вспыхнул, led[1] нет
//   7. Датчик отсутствует: led[3] горит, E001; KEY4 гасит led[3] и обнуляет счётчики
//   8. EE_WRITE (сценарий 4): значение записано в EEPROM и показано на индикаторе; ACK-опрос не ошибка
//   9. Датчик вернулся, DT_TEMP снова «1960»
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps
`include "i2c_line_mon.svh"

module I2C_topB_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  SLOT     = 20;
    localparam int  BLANK    = 2;
    localparam int  FLASH    = 30;
    localparam int  DEB      = 20;
    localparam int  SCROLL   = 20_000;

    // сегменты {g..a} цифр 0..F (как в seg7_dynamic)
    localparam logic [6:0] SEG_REF [16] = '{
        7'h3F, 7'h06, 7'h5B, 7'h4F, 7'h66, 7'h6D, 7'h7D, 7'h07,
        7'h7F, 7'h6F, 7'h77, 7'h7C, 7'h39, 7'h5E, 7'h79, 7'h71
    };

    class Check;
        int errors = 0;

        function void err(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk   = 1'b0;
    logic       rst_n = 1'b0;
    logic [3:0] key   = 4'b1111;
    logic [7:0] seg;
    logic [3:0] dig, led;
    tri1        scl, sda;

    always #(CLK_NS / 2) clk = ~clk;

    I2C_topB #(
        .CLK_FREQ    (CLK_FREQ),
        .DEB_CLKS    (DEB),
        .DIGIT_HZ    (CLK_FREQ / SLOT),
        .BLANK_CLKS  (BLANK),
        .FLASH_CLKS  (FLASH),
        .SCROLL_CLKS (SCROLL)
    ) dut (
        .clk (clk), .rst_n (rst_n), .key (key),
        .i2c_scl (scl), .i2c_sda (sda),
        .seg (seg), .dig (dig), .led (led)
    );

    i2c_slave_model #(.ADDR (7'h48)) s48 (.scl (scl), .sda (sda));
    i2c_slave_model #(
        .ADDR (7'h50), .ADDR_MASK (7'h7C), .MEM_BITS (10), .BLOCK_BITS (2), .PAGE_SIZE (16),
        .WRITE_CYCLE_NS (5_000_000.0)
    ) s50 (.scl (scl), .sda (sda));

    Check      chk = new();
    I2CLineMon mon = new();

    always @(posedge scl) if (rst_n) mon.scl_rise($realtime, sda);
    always @(negedge scl) if (rst_n) mon.scl_fall($realtime, sda);
    always @(posedge sda) if (rst_n) mon.sda_rise($realtime, scl);
    always @(negedge sda) if (rst_n) mon.sda_fall($realtime, scl);

    wire [7:0] seg_on = ~seg;
    wire [3:0] dig_on = ~dig;
    wire [3:0] led_on = ~led;

    // какие светодиоды зажигались (сбрасывается перед сценарием)
    logic [3:0] led_seen = '0;
    always @(posedge clk) led_seen <= led_seen | led_on;

    // -------------------------------------------------------------------------
    // Кнопки
    // -------------------------------------------------------------------------
    task automatic press(input int k, input int hold = 3 * DEB);
        key[k] = 1'b0;
        repeat (hold) @(posedge clk);
        key[k] = 1'b1;
        repeat (DEB + 5) @(posedge clk);
    endtask

    // запустить выбранный сценарий и дождаться конца
    task automatic run_script();
        @(negedge clk);
        led_seen = '0;                  // по спаду clk: не гоняется с always по фронту
        press(0);
        repeat (2) @(posedge clk);
        while (dut.u_seq.busy) @(posedge clk);
        repeat (FLASH + 20) @(posedge clk);
    endtask

    task automatic select_script(input int n);
        repeat (7) begin
            if (dut.script == n) return;
            press(1);
        end
        chk.err($sformatf("script %0d is not reachable", n));
    endtask

    // -------------------------------------------------------------------------
    // Индикатор: картинка с выводов
    // -------------------------------------------------------------------------
    string HEX_CHARS = "0123456789ABCDEF";

    function automatic string seg_char(logic [7:0] f);
        string s;
        s = "?";
        if (f[6:0] === 7'h00) s = " ";
        else begin
            for (int i = 0; i < 16; i++)
                if (f[6:0] === SEG_REF[i]) s = HEX_CHARS.substr(i, i);
            case (f[6:0])
                7'h78:   s = "t";
                7'h54:   s = "n";
                7'h73:   s = "P";
            endcase
        end
        if (f[7]) s = {s, "."};
        return s;
    endfunction

    task automatic read_text(output string text);
        logic [7:0] f[4];
        foreach (f[i]) f[i] = '0;
        repeat (2 * 4 * SLOT) begin
            @(posedge clk);
            for (int i = 0; i < 4; i++)
                if (dig_on[i]) f[i] = seg_on;
        end
        text = {seg_char(f[3]), seg_char(f[2]), seg_char(f[1]), seg_char(f[0])};
    endtask

    task automatic goto_mode(input int m);
        repeat (9) begin
            if (int'(dut.u_ind.mode) == m) return;
            press(2);
        end
        chk.err($sformatf("display mode %0d is not reachable", m));
    endtask

    task automatic expect_text(input string name, input int m, input string want);
        string t;
        goto_mode(m);
        repeat (5) @(posedge clk);
        read_text(t);
        if (t != want) chk.err($sformatf("%s: display \"%s\", expected \"%s\"", name, t, want));
        else           $display("[%0t]    %s: \"%s\"", $time, name, t);
    endtask

    // ожидаемая строка списка найденных для текущего номера (найденные: 48, 50, 51, 52, 53)
    function automatic string found_text(input int idx);
        logic [7:0] list [5] = '{8'h48, 8'h50, 8'h51, 8'h52, 8'h53};
        return {$sformatf("F%0d.", idx + 1), up(list[idx])};
    endfunction

    function automatic string dec3(input int n);
        return $sformatf("%0d%0d%0d", (n / 100) % 10, (n / 10) % 10, n % 10);
    endfunction

    // hex в верхнем регистре, как рисует индикатор
    function automatic string up(input logic [15:0] w);
        string r;
        r = (w > 16'hFF) ? $sformatf("%h", w) : $sformatf("%h", w[7:0]);
        return r.toupper();
    endfunction

    function automatic string hex4(input logic [15:0] w);
        return up(w);
    endfunction

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        string       t;
        logic [15:0] exp_word;
        logic [7:0]  pattern;
        $timeformat(-6, 2, " us", 0);
        for (int i = 0; i < 256; i++)  s48.mem[i] = 8'hEE;
        for (int i = 0; i < 1024; i++) s50.mem[i] = 8'hEE;
        s48.mem[0] = 8'h19;  s48.mem[1] = 8'h60;
        s48.mem[3] = 8'h50;  s48.mem[4] = 8'h00;
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        $display("[%0t] 1. Reset state", $time);
        if (led_on !== 4'b0000) chk.err($sformatf("led = %b after reset", led_on));
        read_text(t);
        if (t != "0000") chk.err($sformatf("display after reset \"%s\"", t));
        if (scl !== 1'b1 || sda !== 1'b1) chk.err("bus is not idle after reset");

        $display("[%0t] 2. KEY2: script 1", $time);
        press(1);
        repeat (5) @(posedge clk);
        read_text(t);
        if (t != "P  1") chk.err($sformatf("after KEY2 display \"%s\", expected \"P  1\"", t));
        else              $display("[%0t]    \"%s\"", $time, t);

        $display("[%0t] 3. KEY1: DT_TEMP", $time);
        run_script();
        exp_word = 16'h1960;
        if (led_seen[0] !== 1'b1 || led_seen[1] !== 1'b1 || led_seen[2] !== 1'b0 || led_seen[3] !== 1'b0)
            chk.err($sformatf("leds seen during DT_TEMP = %b, expected 0011", led_seen));
        expect_text("data", 0, hex4(exp_word));
        expect_text("address", 1, "A 48");
        expect_text("status", 2, "5 80");
        expect_text("transactions", 3, "t002");
        expect_text("NACK", 4, "n000");
        expect_text("errors", 5, "E000");

        $display("[%0t] 4. DT_TOS", $time);
        press(1);
        run_script();
        exp_word = 16'h5000;
        expect_text("data (Tos)", 0, hex4(exp_word));

        $display("[%0t] 5. SCAN", $time);
        select_script(0);
        run_script();
        if (led_seen[3:0] !== 4'b0111) chk.err($sformatf("leds seen during SCAN = %b, expected 0111", led_seen));
        if (int'(dut.u_ind.mode) != 6) chk.err("the display did not switch to the found list");
        // 112 транзакций сканирования + 2 + 2 прежних
        for (int k = 0; k < 5; k++) begin                    // листание: все пять по очереди
            automatic int i0, i1;
            while (int'(dut.u_ind.found_idx) != k) @(posedge clk);
            repeat (200) @(posedge clk);
            i0 = dut.u_ind.found_idx;
            read_text(t);
            i1 = dut.u_ind.found_idx;
            if (i0 == i1 && i0 == k && t != found_text(k))
                chk.err($sformatf("found list \"%s\", expected \"%s\"", t, found_text(k)));
            else
                $display("[%0t]    found list \"%s\"", $time, t);
        end
        expect_text("transactions after SCAN", 3, {"t", dec3(2 + 2 + 112)});
        expect_text("NACK after SCAN", 4, {"n", dec3(107)});
        expect_text("errors after SCAN", 5, "E000");

        $display("[%0t] 6. ABSENT", $time);
        select_script(5);
        run_script();
        if (led_seen[2] !== 1'b1 || led_seen[1] !== 1'b0 || led_seen[3] !== 1'b0)
            chk.err($sformatf("leds seen during ABSENT = %b", led_seen));
        expect_text("NACK after ABSENT", 4, {"n", dec3(108)});
        expect_text("errors after ABSENT", 5, "E000");
        expect_text("status after ABSENT", 2, "5 01");

        $display("[%0t] 7. Sensor is missing, KEY4", $time);
        s48.absent = 1'b1;
        select_script(1);
        run_script();
        if (led_on[3] !== 1'b1) chk.err("led[3] is off after an error");
        expect_text("errors", 5, "E001");
        press(3);
        repeat (5) @(posedge clk);
        if (led_on[3] !== 1'b0) chk.err("led[3] is still on after KEY4");
        expect_text("errors after KEY4", 5, "E000");
        expect_text("transactions after KEY4", 3, "t000");
        expect_text("data after KEY4", 0, "0000");
        s48.absent = 1'b0;

        $display("[%0t] 8. EE_WRITE", $time);
        select_script(4);
        pattern = 8'h5A + 8'(dut.u_seq.run_cnt);
        run_script();
        if (s50.mem[8'hF0] !== pattern) chk.err($sformatf("EEPROM cell F0 = %h, expected %h", s50.mem[8'hF0], pattern));
        expect_text("data (EEPROM read back)", 0, {"00", up(pattern)});
        expect_text("errors after EE_WRITE", 5, "E000");
        if (led_seen[3] !== 1'b0) chk.err("led[3] lit during EE_WRITE (polling NACK is not an error)");

        $display("[%0t] 9. Sensor is back", $time);
        select_script(1);
        run_script();
        expect_text("data", 0, "1960");

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d (testbench) + %0d (bus monitor)", chk.errors, mon.errors);
        if (chk.errors == 0 && mon.errors == 0) $display("TEST PASSED");
        else                                    $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #300ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
