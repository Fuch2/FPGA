// =============================================================================
// Тестбенч для i2c_ind (индикация обмена по I2C)
//
// Параметры уменьшены: слот разряда 20 тактов (гашение 2), вспышка 30 тактов,
// показ одного найденного адреса 4000 тактов.
// Классы:
//   IndModel - эталон: счётчики (по модулю 1000), режим, вспышки светодиодов,
//              листание списка найденных, ожидаемая картинка индикатора.
// Проверки:
//   1. После сброса: режим «0000», светодиоды погашены
//   2. 20000 тактов случайных событий: светодиоды, found_idx и режим совпадают с эталоном в каждом такте
//   3. Все 8 режимов индикатора (next_mode) по кругу; картинка совпадает с эталоном
//   4. Переходы режима: script_changed -> «P   N», run со сканированием -> «Fi.AA», run -> данные
//   5. Список найденных: «F0.00», когда пуст; листание по кругу
//   6. Переход счётчика 999 -> 000
//   7. clear обнуляет счётчики и гасит led[3]
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module i2c_ind_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  SLOT     = 20;
    localparam int  BLANK    = 2;
    localparam int  FLASH    = 30;
    localparam int  SCROLL   = 4000;
    localparam int  N_FOUND  = 8;
    localparam int  N_MODES  = 8;

    // сегменты {g..a} цифр 0..F (как в seg7_dynamic)
    localparam logic [6:0] SEG_REF [16] = '{
        7'h3F, 7'h06, 7'h5B, 7'h4F, 7'h66, 7'h6D, 7'h7D, 7'h07,
        7'h7F, 7'h6F, 7'h77, 7'h7C, 7'h39, 7'h5E, 7'h79, 7'h71
    };
    localparam string NAME [N_MODES] = '{"D", "A", "S", "t", "n", "E", "F", "P"};

    class IndModel;
        int          cnt[3];                    // t, n, E
        int          mode = 0;
        bit          err_seen = 0;
        int          start_flash = 0, ack_flash = 0, nack_flash = 0;
        int          scroll = 0;
        int          found_idx = 0;

        function new();
            foreach (cnt[i]) cnt[i] = 0;
        endfunction

        function void step(bit ev_start, bit ev_ack, bit ev_nack, bit txn_p, bit nack_p, bit err_p,
                           bit run, bit run_scan, bit script_changed, bit clear, bit next_mode, int found_cnt);
            int idx_n = found_idx;
            if (next_mode)      mode = (mode + 1) % N_MODES;
            if (script_changed) mode = 7;
            if (run)            mode = run_scan ? 6 : 0;
            if (clear) begin
                foreach (cnt[i]) cnt[i] = 0;
                err_seen = 0;
            end else begin
                if (txn_p)  cnt[0] = (cnt[0] + 1) % 1000;
                if (nack_p) cnt[1] = (cnt[1] + 1) % 1000;
                if (err_p) begin
                    cnt[2]   = (cnt[2] + 1) % 1000;
                    err_seen = 1;
                end
            end
            if (scroll == SCROLL - 1) begin
                scroll = 0;
                idx_n  = (found_cnt == 0 || found_idx + 1 >= found_cnt) ? 0 : found_idx + 1;
            end else begin
                scroll++;
            end
            if (found_cnt == 0 || found_idx >= found_cnt) idx_n = 0;
            found_idx   = idx_n;
            start_flash = ev_start ? FLASH : (start_flash > 0 ? start_flash - 1 : 0);
            ack_flash   = ev_ack   ? FLASH : (ack_flash   > 0 ? ack_flash   - 1 : 0);
            nack_flash  = ev_nack  ? FLASH : (nack_flash  > 0 ? nack_flash  - 1 : 0);
        endfunction

        function logic [7:0] hexseg(int h);
            return {1'b0, SEG_REF[h & 15]};
        endfunction

        // ожидаемые сегменты разрядов (active 1), lit[i] = 0 - разряд погашен
        function void frame(logic [15:0] word, logic [6:0] addr, logic [7:0] status, int found_cnt,
                            logic [6:0] found_addr, logic [2:0] script,
                            output logic [7:0] f[4], output logic [3:0] lit);
            int v;
            lit = 4'b1111;
            foreach (f[i]) f[i] = '0;
            case (mode)
                0: for (int i = 0; i < 4; i++) f[i] = hexseg((word >> (4 * i)) & 4'hF);
                1: begin
                    f[3] = hexseg(10);  lit[2] = 1'b0;
                    f[1] = hexseg(addr >> 4);  f[0] = hexseg(addr & 4'hF);
                end
                2: begin
                    f[3] = 8'h6D;  lit[2] = 1'b0;
                    f[1] = hexseg(status >> 4);  f[0] = hexseg(status & 4'hF);
                end
                3, 4, 5: begin
                    v    = cnt[mode - 3];
                    f[3] = (mode == 3) ? 8'h78 : (mode == 4) ? 8'h54 : 8'h79;
                    f[2] = hexseg((v / 100) % 10);
                    f[1] = hexseg((v / 10) % 10);
                    f[0] = hexseg(v % 10);
                end
                6: begin
                    f[3] = hexseg(15);
                    f[2] = hexseg(found_cnt == 0 ? 0 : found_idx + 1) | 8'h80;      // с десятичной точкой
                    f[1] = hexseg(found_addr >> 4);  f[0] = hexseg(found_addr & 4'hF);
                end
                default: begin
                    f[3] = 8'h73;  lit[2:1] = 2'b00;
                    f[0] = hexseg(script);
                end
            endcase
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic        clk = 1'b0, rst_n = 1'b0;
    logic        ev_start = 0, ev_ack = 0, ev_nack = 0, txn_p = 0, nack_p = 0, err_p = 0;
    logic [15:0] info_word = '0;
    logic [6:0]  info_addr = '0;
    logic [7:0]  info_status = '0;
    logic [$clog2(N_FOUND):0] found_cnt = '0;
    logic [6:0]  found_addr;
    logic [$clog2(N_FOUND)-1:0] found_idx;
    logic [2:0]  script = '0;
    logic        run = 0, run_scan = 0, script_changed = 0, clear = 0, next_mode = 0;
    logic [7:0]  seg;
    logic [3:0]  dig, led;

    logic [6:0]  found_mem [N_FOUND];

    always #(CLK_NS / 2) clk = ~clk;

    assign found_addr = found_mem[found_idx];

    i2c_ind #(
        .CLK_FREQ    (CLK_FREQ),
        .DIGIT_HZ    (CLK_FREQ / SLOT),
        .BLANK_CLKS  (BLANK),
        .FLASH_CLKS  (FLASH),
        .SCROLL_CLKS (SCROLL),
        .N_FOUND     (N_FOUND)
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

    // светодиоды, режим и found_idx - каждый такт
    always @(posedge clk) begin
        if (rst_n) begin
            automatic logic [3:0] exp = {model.err_seen, model.nack_flash != 0, model.ack_flash != 0, model.start_flash != 0};
            if (led_on !== exp) fail($sformatf("led = %b, expected %b", led_on, exp));
            if (int'(found_idx) !== model.found_idx) fail($sformatf("found_idx = %0d, expected %0d", found_idx, model.found_idx));
            if (int'(dut.mode) !== model.mode) fail($sformatf("mode = %0d, expected %0d", dut.mode, model.mode));
            model.step(ev_start, ev_ack, ev_nack, txn_p, nack_p, err_p, run, run_scan, script_changed, clear,
                       next_mode, int'(found_cnt));
        end
    end

    // снять картинку индикатора: по 2 раза каждый слот
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

    task automatic check_display();
        logic [7:0] f[4], e[4];
        logic [3:0] lit, e_lit;
        while (model.scroll > SCROLL - 400 || model.scroll < 3) @(posedge clk);    // не на смене найденного адреса
        repeat (5) @(posedge clk);
        model.frame(info_word, info_addr, info_status, int'(found_cnt), found_addr, script, e, e_lit);
        read_display(f, lit);
        if (lit !== e_lit) fail($sformatf("mode %s: lit digits %b, expected %b", NAME[model.mode], lit, e_lit));
        for (int i = 0; i < 4; i++)
            if (e_lit[i] && f[i] !== e[i])
                fail($sformatf("mode %s: digit %0d = %h, expected %h", NAME[model.mode], i, f[i], e[i]));
        $display("[%0t]    mode %s: %h %h %h %h (dig3..dig0)", $time, NAME[model.mode], f[3], f[2], f[1], f[0]);
    endtask

    localparam int P_RUN = 0, P_SCRIPT = 1, P_NEXT = 2, P_CLEAR = 3, P_TXN = 4, P_ERR = 5;

    // импульс на 1 такт на одном из управляющих входов
    task automatic pulse(input int which);
        case (which)
            P_RUN:    run            <= 1'b1;
            P_SCRIPT: script_changed <= 1'b1;
            P_NEXT:   next_mode      <= 1'b1;
            P_CLEAR:  clear          <= 1'b1;
            P_TXN:    txn_p          <= 1'b1;
            default:  err_p          <= 1'b1;
        endcase
        @(posedge clk);
        {run, script_changed, next_mode, clear, txn_p, err_p} <= '0;
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        $timeformat(-6, 2, " us", 0);
        foreach (found_mem[i]) found_mem[i] = 7'h40 + 7'(i) * 7'd3;
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        $display("[%0t] 1. Reset state", $time);
        check_display();

        $display("[%0t] 2. 20000 clocks of random events", $time);
        repeat (20000) begin
            ev_start       <= ($urandom_range(0, 19) == 0);
            ev_ack         <= ($urandom_range(0, 19) == 0);
            ev_nack        <= ($urandom_range(0, 19) == 0);
            txn_p          <= ($urandom_range(0, 14) == 0);
            nack_p         <= ($urandom_range(0, 14) == 0);
            err_p          <= ($urandom_range(0, 29) == 0);
            info_word      <= $urandom;
            info_addr      <= $urandom;
            info_status    <= $urandom;
            found_cnt      <= $urandom_range(0, N_FOUND);
            script         <= $urandom_range(0, 5);
            run            <= ($urandom_range(0, 399) == 0);
            run_scan       <= $urandom;
            script_changed <= ($urandom_range(0, 399) == 0);
            next_mode      <= ($urandom_range(0, 399) == 0);
            clear          <= ($urandom_range(0, 1999) == 0);
            @(posedge clk);
        end
        {ev_start, ev_ack, ev_nack, txn_p, nack_p, err_p, run, script_changed, next_mode, clear} <= '0;
        repeat (3) @(posedge clk);

        $display("[%0t] 3. All display modes", $time);
        found_cnt <= 5;
        info_word <= 16'h1960;  info_addr <= 7'h48;  info_status <= 8'h81;  script <= 3'd4;
        pulse(P_RUN);  repeat (3) @(posedge clk);              // режим 0 и данные
        repeat (N_MODES) begin
            check_display();
            pulse(P_NEXT);
        end
        check_display();

        $display("[%0t] 4. Mode jumps", $time);
        pulse(P_SCRIPT);
        repeat (3) @(posedge clk);
        if (model.mode != 7) fail("script_changed did not select the P mode");
        check_display();
        run_scan <= 1'b1;
        pulse(P_RUN);
        repeat (3) @(posedge clk);
        if (model.mode != 6) fail("run with scan did not select the F mode");
        check_display();
        run_scan <= 1'b0;
        pulse(P_RUN);
        repeat (3) @(posedge clk);
        if (model.mode != 0) fail("run did not select the data mode");
        check_display();

        $display("[%0t] 5. Found list", $time);
        run_scan <= 1'b1;
        pulse(P_RUN);
        found_cnt <= 0;
        repeat (3) @(posedge clk);
        check_display();                                     // «F0.00»
        found_cnt <= 3;
        for (int k = 0; k < 4; k++) begin                    // листание 1, 2, 3, 1
            repeat (SCROLL) @(posedge clk);
            check_display();
        end
        run_scan <= 1'b0;

        $display("[%0t] 6. Counter 999 -> 000", $time);
        repeat (3) pulse(P_NEXT);                         // режим t (после режима 6 -> 7 -> 0 -> ...)
        while (model.mode != 3) pulse(P_NEXT);
        txn_p <= 1'b1;
        repeat ((998 - model.cnt[0] + 1000) % 1000) @(posedge clk);
        txn_p <= 1'b0;
        check_display();                                     // t998
        repeat (2) begin
            pulse(P_TXN);
            check_display();                                 // t999, t000
        end

        $display("[%0t] 7. clear", $time);
        pulse(P_ERR);
        repeat (3) @(posedge clk);
        if (!model.err_seen || led_on[3] !== 1'b1) fail("led[3] is off after an error");
        pulse(P_CLEAR);
        repeat (3) @(posedge clk);
        if (led_on[3] !== 1'b0) fail("led[3] is still on after clear");
        repeat (N_MODES) begin
            check_display();
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
        #10ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
