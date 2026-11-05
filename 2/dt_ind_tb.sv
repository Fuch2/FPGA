// =============================================================================
// Тестбенч для dt_ind (индикация чтения ДТ: температура в градусах, светодиоды)
//
// Параметры уменьшены: слот разряда 20 тактов (гашение 2), вспышка 30 тактов.
// Класс IndModel - эталон: счётчики (по модулю 1000), режим, флаги last_ok / have_value,
// вспышки светодиодов и ожидаемая картинка индикатора; температура переводится
// в десятые доли градуса в целых числах, независимо от temp_fmt.
// Проверки:
//   1. После сброса: режим «температура», на индикаторе «----», светодиоды погашены
//   2. 20000 тактов случайных событий: светодиоды, режим, счётчики и флаги совпадают с
//      эталоном в каждом такте
//   3. Картинка температуры: 0.0, 0.1, 5.3, 25.4, 100.0, 125.0, -0.1, -5.3, -25.4, -55.0,
//      ограничение -99.9; «Err» после неудачи, возврат после удачного чтения
//   4. Все режимы по кругу (next_mode); show_temp и show_src переключают режим
//   5. Переход счётчика 999 -> 000
//   6. clear обнуляет счётчики и гасит led[3]
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module dt_ind_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  SLOT     = 20;
    localparam int  BLANK    = 2;
    localparam int  FLASH    = 30;
    localparam int  N_MODES  = 5;
    localparam int  SETTLE   = 20;          // тактов до готовности цифр после temp_valid

    localparam logic [6:0] SEG_REF [16] = '{
        7'h3F, 7'h06, 7'h5B, 7'h4F, 7'h66, 7'h6D, 7'h7D, 7'h07,
        7'h7F, 7'h6F, 7'h77, 7'h7C, 7'h39, 7'h5E, 7'h79, 7'h71
    };
    localparam string NAME [N_MODES] = '{"T", "H", "r", "E", "S"};

    class IndModel;
        int          cnt_ok = 0, cnt_fail = 0;
        int          mode = 0;
        bit          fail_seen = 0, last_ok = 1, have_value = 0;
        logic [15:0] raw = '0;
        int          start_flash = 0, ack_flash = 0, nack_flash = 0;
        int          fmt_delay = -1;                    // тактов до готовности цифр

        function void step(bit ev_start, bit ev_ack, bit ev_nack, bit temp_valid, logic [15:0] data,
                           bit read_fail, bit clear, bit next_mode, bit show_temp, bit show_src);
            if (next_mode) mode = (mode + 1) % N_MODES;
            if (show_temp) mode = 0;
            if (show_src)  mode = 4;
            if (temp_valid) begin
                raw       = data;
                last_ok   = 1;
                fmt_delay = SETTLE;
            end
            if (fmt_delay > 0) fmt_delay--;
            if (fmt_delay == 0) begin have_value = 1; fmt_delay = -1; end
            if (clear) begin
                cnt_ok = 0;  cnt_fail = 0;  fail_seen = 0;
            end else begin
                if (temp_valid) cnt_ok = (cnt_ok + 1) % 1000;
                if (read_fail) begin
                    cnt_fail  = (cnt_fail + 1) % 1000;
                    fail_seen = 1;
                end
            end
            if (read_fail) last_ok = 0;
            start_flash = ev_start ? FLASH : (start_flash > 0 ? start_flash - 1 : 0);
            ack_flash   = ev_ack   ? FLASH : (ack_flash   > 0 ? ack_flash   - 1 : 0);
            nack_flash  = ev_nack  ? FLASH : (nack_flash  > 0 ? nack_flash  - 1 : 0);
        endfunction

        function logic [7:0] hexseg(int h);
            return {1'b0, SEG_REF[h & 15]};
        endfunction

        // ожидаемые сегменты разрядов (active 1), lit[i] = 0 - разряд погашен
        function void frame(bit src_model, int gen_mode, output logic [7:0] f[4], output logic [3:0] lit);
            int r, mag, tenths, h, t, u, d;
            bit neg;
            lit = 4'b1111;
            foreach (f[i]) f[i] = '0;
            case (mode)
                0: begin
                    r      = $signed(raw[15:5]);
                    neg    = (r < 0);
                    mag    = neg ? -r : r;
                    tenths = (mag * 5 + 2) / 4;
                    if (neg && tenths > 999) tenths = 999;
                    h = tenths / 1000;  t = (tenths / 100) % 10;  u = (tenths / 10) % 10;  d = tenths % 10;
                    if (!last_ok) begin
                        lit[3] = 1'b0;
                        f[2] = 8'h79;  f[1] = 8'h50;  f[0] = 8'h50;
                    end else if (!have_value) begin
                        foreach (f[i]) f[i] = 8'h40;
                    end else begin
                        f[3] = hexseg(h);  f[2] = hexseg(t);  f[1] = hexseg(u) | 8'h80;  f[0] = hexseg(d);
                        if (neg) begin
                            if (h != 0 || t != 0) f[3] = 8'h40;
                            else begin lit[3] = 1'b0;  f[2] = 8'h40; end
                        end else begin
                            if (h == 0) lit[3] = 1'b0;
                            if (h == 0 && t == 0) lit[2] = 1'b0;
                        end
                    end
                end
                1: for (int i = 0; i < 4; i++) f[i] = hexseg((raw >> (4 * i)) & 4'hF);
                2, 3: begin
                    int v;
                    v = (mode == 2) ? cnt_ok : cnt_fail;
                    f[3] = (mode == 2) ? 8'h50 : 8'h79;
                    f[2] = hexseg((v / 100) % 10);
                    f[1] = hexseg((v / 10) % 10);
                    f[0] = hexseg(v % 10);
                end
                default: begin
                    f[3] = 8'h6D;  lit[1] = 1'b0;
                    f[2] = hexseg(src_model ? 1 : 0);
                    f[0] = hexseg(gen_mode);
                end
            endcase
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic        clk = 1'b0, rst_n = 1'b0;
    logic        ev_start = 0, ev_ack = 0, ev_nack = 0, temp_valid = 0, read_fail = 0;
    logic [15:0] temp_data = '0;
    logic        src_model = 0;
    logic [1:0]  gen_mode = '0;
    logic        clear = 0, next_mode = 0, show_temp = 0, show_src = 0;
    logic [7:0]  seg;
    logic [3:0]  dig, led;

    always #(CLK_NS / 2) clk = ~clk;

    dt_ind #(
        .CLK_FREQ   (CLK_FREQ),
        .DIGIT_HZ   (CLK_FREQ / SLOT),
        .BLANK_CLKS (BLANK),
        .FLASH_CLKS (FLASH)
    ) dut (.*);

    wire [7:0] seg_on = ~seg;
    wire [3:0] dig_on = ~dig;
    wire [3:0] led_on = ~led;

    IndModel model  = new();
    int      errors = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    function automatic int bcd_to_int(logic [11:0] b);
        return b[11:8] * 100 + b[7:4] * 10 + b[3:0];
    endfunction

    // светодиоды, режим, счётчики, флаги - каждый такт
    always @(posedge clk) begin
        if (rst_n) begin
            automatic logic [3:0] exp = {model.fail_seen, model.nack_flash != 0, model.ack_flash != 0, model.start_flash != 0};
            if (led_on !== exp) fail($sformatf("led = %b, expected %b", led_on, exp));
            if (int'(dut.mode) !== model.mode) fail($sformatf("mode = %0d, expected %0d", dut.mode, model.mode));
            if (bcd_to_int(dut.cnt_ok) != model.cnt_ok || bcd_to_int(dut.cnt_fail) != model.cnt_fail)
                fail($sformatf("counters ok %0d fail %0d, expected %0d %0d", bcd_to_int(dut.cnt_ok),
                               bcd_to_int(dut.cnt_fail), model.cnt_ok, model.cnt_fail));
            if (dut.last_ok !== model.last_ok) fail("last_ok differs");
            model.step(ev_start, ev_ack, ev_nack, temp_valid, temp_data, read_fail, clear, next_mode, show_temp, show_src);
        end
    end

    task automatic read_display(output logic [7:0] f[4], output logic [3:0] lit);
        lit = '0;
        foreach (f[i]) f[i] = 'x;
        repeat (2 * 4 * SLOT) begin
            @(posedge clk);
            if ($countones(dig_on) > 1) fail($sformatf("several digits on: %b", dig_on));
            for (int i = 0; i < 4; i++)
                if (dig_on[i]) begin
                    if (lit[i] && f[i] !== seg_on) fail($sformatf("digit %0d flickers", i));
                    lit[i] = 1'b1;
                    f[i]   = seg_on;
                end
        end
    endtask

    task automatic check_display(input string what = "");
        logic [7:0] f[4], e[4];
        logic [3:0] lit, e_lit;
        repeat (SETTLE + 5) @(posedge clk);
        model.frame(src_model, int'(gen_mode), e, e_lit);
        read_display(f, lit);
        if (lit !== e_lit) fail($sformatf("%s mode %s: lit digits %b, expected %b", what, NAME[model.mode], lit, e_lit));
        for (int i = 0; i < 4; i++)
            if (e_lit[i] && f[i] !== e[i])
                fail($sformatf("%s mode %s: digit %0d = %h, expected %h", what, NAME[model.mode], i, f[i], e[i]));
        $display("[%0t]    %s mode %s: %h %h %h %h (dig3..dig0)", $time, what, NAME[model.mode], f[3], f[2], f[1], f[0]);
    endtask

    localparam int P_VALID = 0, P_FAIL = 1, P_NEXT = 2, P_CLEAR = 3, P_TEMP = 4, P_SRC = 5;

    task automatic pulse(input int which);
        case (which)
            P_VALID: temp_valid <= 1'b1;
            P_FAIL:  read_fail  <= 1'b1;
            P_NEXT:  next_mode  <= 1'b1;
            P_CLEAR: clear      <= 1'b1;
            P_TEMP:  show_temp  <= 1'b1;
            default: show_src   <= 1'b1;
        endcase
        @(posedge clk);
        {temp_valid, read_fail, next_mode, clear, show_temp, show_src} <= '0;
    endtask

    task automatic show_temperature(input string name, input logic [15:0] t);
        temp_data <= t;
        pulse(P_VALID);
        check_display(name);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        int gap;
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        $display("[%0t] 1. Reset state", $time);
        check_display("reset");

        $display("[%0t] 2. 20000 clocks of random events", $time);
        gap = 0;
        begin
            repeat (20000) begin
                ev_start   <= ($urandom_range(0, 19) == 0);
                ev_ack     <= ($urandom_range(0, 19) == 0);
                ev_nack    <= ($urandom_range(0, 19) == 0);
                // результаты чтения - не чаще, чем раз в 2 * SETTLE тактов (как в системе: раз в сотни мс)
                if (gap > 0) begin
                    gap--;
                    temp_valid <= 1'b0;
                    read_fail  <= 1'b0;
                end else begin
                    temp_valid <= ($urandom_range(0, 59) == 0);
                    read_fail  <= ($urandom_range(0, 79) == 0) && !temp_valid;
                    gap = 2 * SETTLE;
                end
                temp_data  <= $urandom;
                src_model  <= $urandom;
                gen_mode   <= $urandom;
                next_mode  <= ($urandom_range(0, 199) == 0);
                show_temp  <= ($urandom_range(0, 399) == 0);
                show_src   <= ($urandom_range(0, 399) == 0);
                clear      <= ($urandom_range(0, 1999) == 0);
                @(posedge clk);
            end
        end
        {ev_start, ev_ack, ev_nack, temp_valid, read_fail, next_mode, show_temp, show_src, clear} <= '0;
        repeat (3) @(posedge clk);

        $display("[%0t] 3. Temperature pictures", $time);
        pulse(P_TEMP);
        show_temperature("25.375",  16'h1960);                 // 25.4
        show_temperature("0.0",     16'h0000);
        show_temperature("0.125",   16'h0020);                 // 0.1
        show_temperature("5.25",    16'h0540);                 // 5.3
        show_temperature("100.0",   16'h6400);
        show_temperature("125.0",   16'h7D00);
        show_temperature("-0.125",  16'hFFE0);                 // -0.1
        show_temperature("-5.25",   16'hFAC0);                 // -5.3
        show_temperature("-25.375", 16'hE6A0);                 // -25.4
        show_temperature("-55.0",   {11'(-440), 5'b0});
        show_temperature("-128",    16'h8000);                 // -99.9
        pulse(P_FAIL);
        check_display("after fail");                           // « Err»
        show_temperature("25.375 again", 16'h1960);

        $display("[%0t] 4. All display modes", $time);
        src_model <= 1'b1;  gen_mode <= 2'd2;
        pulse(P_TEMP);
        repeat (N_MODES) begin
            check_display("loop");
            pulse(P_NEXT);
        end
        pulse(P_SRC);
        check_display("show_src");
        if (model.mode != 4) fail("show_src did not select the source mode");
        pulse(P_TEMP);
        check_display("show_temp");
        if (model.mode != 0) fail("show_temp did not select the temperature mode");

        $display("[%0t] 5. Counter 999 -> 000", $time);
        while (model.mode != 2) pulse(P_NEXT);
        repeat ((998 - model.cnt_ok + 1000) % 1000) begin
            temp_valid <= 1'b1;
            @(posedge clk);
        end
        temp_valid <= 1'b0;
        check_display("r998");
        repeat (2) begin
            pulse(P_VALID);
            check_display("r999 / r000");
        end

        $display("[%0t] 6. clear", $time);
        pulse(P_FAIL);
        repeat (3) @(posedge clk);
        if (!model.fail_seen || led_on[3] !== 1'b1) fail("led[3] is off after a failed read");
        pulse(P_CLEAR);
        repeat (3) @(posedge clk);
        if (led_on[3] !== 1'b0) fail("led[3] is still on after clear");
        repeat (N_MODES) begin
            check_display("after clear");
            pulse(P_NEXT);
        end

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d", errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #20ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
