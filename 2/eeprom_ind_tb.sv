// =============================================================================
// Тестбенч для eeprom_ind (индикация проверки EEPROM: адрес, данные, указатели, светодиоды)
//
// Параметры уменьшены: слот разряда 20 тактов (гашение 2), вспышка 30 тактов, экран
// режима АВТО 2000 тактов. Класс IndModel - эталон: режим, счётчик ошибок, чередование
// экранов АВТО и вспышки светодиодов; картинка индикатора считается функцией frame()
// независимо от eeprom_ind (десятичные цифры - делением).
// Проверки:
//   1. После сброса: режим АВТО, светодиоды погашены
//   2. 20000 тактов случайных событий: светодиоды, режим, счётчик ошибок и чередование
//      экранов АВТО совпадают с эталоном в каждом такте
//   3. Картинка каждого режима при заданных входах: данные «1960», «----», адрес «A.2A4»,
//      курсор «c.155», голова «H.1FF», хвост «t.0A5» (точка в младшем разряде при ovf),
//      записей «n.512», опросов «P.045», ошибок «E.003», сценарий «S.   5»
//   4. Десятичные счётчики 0, 7, 99, 100, 512 и 0, 9, 10, 255 (последовательный bin2bcd)
//   5. Режим АВТО чередует экраны адреса и данных; next_mode проходит режимы 0..8 по кругу,
//      режим сценария в этот круг не входит; run возвращает режим АВТО
//   6. clear обнуляет счётчик ошибок и гасит led[0]
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module eeprom_ind_tb;

    localparam int  CLK_FREQ  = 50_000_000;
    localparam real CLK_NS    = 1.0e9 / CLK_FREQ;
    localparam int  SLOT      = 20;
    localparam int  BLANK     = 2;
    localparam int  FLASH     = 30;
    localparam int  AUTO      = 2000;
    localparam int  SETTLE    = 60;             // тактов до готовности десятичных цифр

    localparam logic [6:0] SEG_REF [16] = '{
        7'h3F, 7'h06, 7'h5B, 7'h4F, 7'h66, 7'h6D, 7'h7D, 7'h07,
        7'h7F, 7'h6F, 7'h77, 7'h7C, 7'h39, 7'h5E, 7'h79, 7'h71
    };
    localparam logic [7:0] DP = 8'h80;
    localparam string NAME [10] = '{"auto", "data", "addr", "cursor", "head", "tail", "count", "polls", "errors", "script"};

    class IndModel;
        int mode = 0;
        int cnt_err = 0;
        bit err_seen = 0;
        int auto_cnt = 0;
        bit auto_data = 0;
        int wr_f = 0, rd_f = 0, busy_f = 0;

        function void step(bit next_mode, bit script_changed, bit run, bit clear, bit err_p, bit wr_p, bit rd_p, bit polling);
            if (next_mode)      mode = (mode >= 8) ? 0 : mode + 1;
            if (script_changed) mode = 9;
            if (run)            mode = 0;
            if (clear) begin
                cnt_err = 0;  err_seen = 0;
            end else if (err_p) begin
                cnt_err = (cnt_err + 1) % 1000;
                err_seen = 1;
            end
            if (auto_cnt == AUTO - 1) begin
                auto_cnt  = 0;
                auto_data = !auto_data;
            end else begin
                auto_cnt++;
            end
            wr_f   = wr_p    ? FLASH : (wr_f   > 0 ? wr_f   - 1 : 0);
            rd_f   = rd_p    ? FLASH : (rd_f   > 0 ? rd_f   - 1 : 0);
            busy_f = polling ? FLASH : (busy_f > 0 ? busy_f - 1 : 0);
        endfunction

        function int screen();
            return (mode == 0) ? (auto_data ? 1 : 2) : mode;
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic        clk = 1'b0, rst_n = 1'b0;
    logic [9:0]  info_addr = '0, cursor = '0, ring_count = '0;
    logic [15:0] info_data = '0;
    logic        info_empty = 1'b0, ring_ovf = 1'b0;
    logic [8:0]  ring_wr_ptr = '0, ring_rd_ptr = '0;
    logic [7:0]  polls_last = '0;
    logic [3:0]  script = '0;
    logic        wr_p = 0, rd_p = 0, err_p = 0, polling = 0;
    logic        next_mode = 0, script_changed = 0, run = 0, clear = 0;
    logic [7:0]  seg;
    logic [3:0]  dig, led;

    always #(CLK_NS / 2) clk = ~clk;

    eeprom_ind #(
        .CLK_FREQ   (CLK_FREQ),
        .ADDR_W     (10),
        .COUNT_W    (10),
        .PTR_W      (9),
        .AUTO_CLKS  (AUTO),
        .DIGIT_HZ   (CLK_FREQ / SLOT),
        .BLANK_CLKS (BLANK),
        .FLASH_CLKS (FLASH)
    ) dut (
        .clk (clk), .rst_n (rst_n),
        .info_addr (info_addr), .info_data (info_data), .info_empty (info_empty), .cursor (cursor),
        .ring_count (ring_count), .ring_wr_ptr (ring_wr_ptr), .ring_rd_ptr (ring_rd_ptr), .ring_ovf (ring_ovf),
        .polls_last (polls_last), .script (script),
        .wr_p (wr_p), .rd_p (rd_p), .err_p (err_p), .polling (polling),
        .next_mode (next_mode), .script_changed (script_changed), .run (run), .clear (clear),
        .seg (seg), .dig (dig), .led (led)
    );

    wire [7:0] seg_on = ~seg;
    wire [3:0] dig_on = ~dig;
    wire [3:0] led_on = ~led;

    IndModel model  = new();
    int      errors = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    function automatic logic [7:0] hexseg(int h);
        return {1'b0, SEG_REF[h & 15]};
    endfunction

    // светодиоды, режим, счётчик ошибок, чередование экранов - каждый такт
    function automatic int bcd3_to_int(logic [11:0] b);
        return b[11:8] * 100 + b[7:4] * 10 + b[3:0];
    endfunction

    always @(posedge clk) begin
        if (rst_n) begin
            automatic logic [3:0] exp = {model.wr_f != 0, model.rd_f != 0, model.busy_f != 0, model.err_seen};
            if (led_on !== exp) fail($sformatf("led = %b, expected %b", led_on, exp));
            if (int'(dut.mode) !== model.mode) fail($sformatf("mode = %0d, expected %0d", dut.mode, model.mode));
            if (bcd3_to_int(dut.cnt_err) != model.cnt_err)
                fail($sformatf("error counter %0d, expected %0d", bcd3_to_int(dut.cnt_err), model.cnt_err));
            if (dut.auto_data !== model.auto_data) fail("auto_data differs");
            model.step(next_mode, script_changed, run, clear, err_p, wr_p, rd_p, polling);
        end
    end

    // ожидаемая картинка: f[i] - сегменты разряда i (active 1), lit[i] = 0 - разряд погашен
    function automatic void frame(output logic [7:0] f[4], output logic [3:0] lit);
        int scr, v;
        lit = 4'b1111;
        foreach (f[i]) f[i] = '0;
        scr = model.screen();
        case (scr)
            1: begin                                        // данные
                if (info_empty) foreach (f[i]) f[i] = 8'h40;
                else for (int i = 0; i < 4; i++) f[i] = hexseg((info_data >> (4 * i)) & 4'hF);
            end
            2, 3, 4, 5: begin                               // адрес, курсор, голова, хвост: буква и три цифры hex
                case (scr)
                    2: begin v = info_addr;    f[3] = 8'h77 | DP; end
                    3: begin v = cursor;       f[3] = 8'h58 | DP; end
                    4: begin v = ring_wr_ptr;  f[3] = 8'h76 | DP; end
                    default: begin v = ring_rd_ptr; f[3] = 8'h78 | DP; end
                endcase
                f[2] = hexseg((v >> 8) & 15);  f[1] = hexseg((v >> 4) & 15);  f[0] = hexseg(v & 15);
                if (scr >= 4 && ring_ovf) f[0] |= DP;
            end
            6, 7, 8: begin                                  // записей, опросов, ошибок: буква и три десятичные цифры
                case (scr)
                    6: begin v = ring_count;   f[3] = 8'h54 | DP; end
                    7: begin v = polls_last;   f[3] = 8'h73 | DP; end
                    default: begin v = model.cnt_err; f[3] = 8'h79 | DP; end
                endcase
                f[2] = hexseg((v / 100) % 10);  f[1] = hexseg((v / 10) % 10);  f[0] = hexseg(v % 10);
                if (scr == 6 && ring_ovf) f[0] |= DP;
            end
            default: begin                                  // сценарий «S.   N»
                f[3] = 8'h6D | DP;
                lit[2] = 1'b0;  lit[1] = 1'b0;
                f[0] = hexseg(script);
            end
        endcase
    endfunction

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

    task automatic check_display(input string what);
        logic [7:0] f[4], e[4];
        logic [3:0] lit, e_lit;
        repeat (SETTLE) @(posedge clk);
        if (model.mode == 0 && model.auto_cnt > AUTO - 300) begin    // не читать на стыке экранов АВТО
            while (model.auto_cnt != 5) @(posedge clk);
        end
        frame(e, e_lit);
        read_display(f, lit);
        if (lit !== e_lit) fail($sformatf("%s mode %s: lit digits %b, expected %b", what, NAME[model.mode], lit, e_lit));
        for (int i = 0; i < 4; i++)
            if (e_lit[i] && f[i] !== e[i])
                fail($sformatf("%s mode %s: digit %0d = %h, expected %h", what, NAME[model.mode], i, f[i], e[i]));
        $display("[%0t]    %s mode %s: %h %h %h %h (dig3..dig0)", $time, what, NAME[model.mode], f[3], f[2], f[1], f[0]);
    endtask

    task automatic pulse_next();
        next_mode <= 1'b1;
        @(posedge clk);
        next_mode <= 1'b0;
    endtask

    task automatic pulse_err();
        err_p <= 1'b1;
        @(posedge clk);
        err_p <= 1'b0;
        repeat (3) @(posedge clk);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        $display("[%0t] 1. Reset state", $time);
        repeat (5) @(posedge clk);
        if (led_on !== 4'b0000) fail($sformatf("led after reset = %b", led_on));
        if (model.mode != 0) fail("mode after reset");

        $display("[%0t] 2. 20000 clocks of random events", $time);
        repeat (20000) begin
            wr_p           <= ($urandom_range(0, 39) == 0);
            rd_p           <= ($urandom_range(0, 39) == 0);
            polling        <= ($urandom_range(0, 59) == 0);
            err_p          <= ($urandom_range(0, 149) == 0);
            next_mode      <= ($urandom_range(0, 199) == 0);
            script_changed <= ($urandom_range(0, 499) == 0);
            run            <= ($urandom_range(0, 499) == 0);
            clear          <= ($urandom_range(0, 1999) == 0);
            info_data      <= $urandom;
            @(posedge clk);
        end
        {wr_p, rd_p, polling, err_p, next_mode, script_changed, run, clear} <= '0;
        repeat (5) @(posedge clk);
        run <= 1'b1;  @(posedge clk);  run <= 1'b0;
        clear <= 1'b1;  @(posedge clk);  clear <= 1'b0;
        repeat (FLASH + 5) @(posedge clk);

        $display("[%0t] 3. Pictures of the modes", $time);
        info_data <= 16'h1960;  info_addr <= 10'h2A4;  cursor <= 10'h155;
        ring_wr_ptr <= 9'h1FF;  ring_rd_ptr <= 9'h0A5;  ring_ovf <= 1'b1;
        ring_count <= 10'd512;  polls_last <= 8'd45;  script <= 4'd5;
        repeat (3) pulse_err();
        // АВТО: два экрана (адрес и данные) сменяют друг друга
        begin
            int scr0;
            check_display("auto 1");
            scr0 = model.screen();
            while (model.screen() == scr0) @(posedge clk);
            check_display("auto 2");
            if (model.screen() == scr0) fail("auto mode did not change the screen");
            if (scr0 != 1 && scr0 != 2) fail($sformatf("auto mode shows screen %0d", scr0));
        end
        for (int m = 1; m <= 8; m++) begin
            pulse_next();
            check_display("next_mode");
        end
        pulse_next();
        if (model.mode != 0) fail("next_mode did not return to the auto mode");
        script_changed <= 1'b1;  @(posedge clk);  script_changed <= 1'b0;
        check_display("script changed");
        if (model.mode != 9) fail("script_changed did not select the script mode");
        pulse_next();
        if (model.mode != 0) fail("next_mode from the script mode did not return to the auto mode");
        pulse_next();                                        // данные
        info_empty <= 1'b1;
        check_display("empty");
        info_empty <= 1'b0;
        ring_ovf   <= 1'b0;
        check_display("data, no ovf");
        pulse_next();  pulse_next();  pulse_next();          // курсор, голова, хвост
        check_display("tail, no ovf");

        $display("[%0t] 4. Decimal values", $time);
        for (int k = 0; k < 5; k++) begin
            int cnt_v[5];
            cnt_v = '{0, 7, 99, 100, 512};
            ring_count <= 10'(cnt_v[k]);
            while (model.mode != 6) pulse_next();
            check_display("count");
        end
        for (int k = 0; k < 4; k++) begin
            int pol_v[4];
            pol_v = '{0, 9, 10, 255};
            polls_last <= 8'(pol_v[k]);
            while (model.mode != 7) pulse_next();
            check_display("polls");
        end
        while (model.mode != 8) pulse_next();
        check_display("errors 3");

        $display("[%0t] 5. run and clear", $time);
        run <= 1'b1;  @(posedge clk);  run <= 1'b0;
        if (model.mode != 0) fail("run did not select the auto mode");
        if (led_on[0] !== 1'b1) fail("led[0] (ERR) is off before clear");
        clear <= 1'b1;  @(posedge clk);  clear <= 1'b0;
        repeat (3) @(posedge clk);
        if (led_on[0] !== 1'b0) fail("led[0] (ERR) is on after clear");
        while (model.mode != 8) pulse_next();
        check_display("errors 0");

        repeat (10) @(posedge clk);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

endmodule
