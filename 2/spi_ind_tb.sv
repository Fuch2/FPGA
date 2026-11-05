// =============================================================================
// Тестбенч для spi_ind (индикация обмена по SPI)
//
// Параметры уменьшены: слот разряда 20 тактов (гашение 2), вспышка 30 тактов.
// Классы:
//   IndModel - эталон: счётчики (по модулю 1000), запомненные D / команда / STATUS,
//              режим, вспышки светодиодов по тактам; ожидаемая картинка индикатора.
// Проверки:
//   1. После сброса: режим «0000», светодиоды погашены
//   2. 3000 тактов случайных событий: светодиоды совпадают с эталоном в каждом такте
//   3. Все 6 режимов индикатора (next_mode), возврат к режиму 0
//   4. Переход счётчика 999 -> 000
//   5. clear обнуляет счётчики, D, команду, STATUS и гасит led[3]
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module spi_ind_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  SLOT     = 20;
    localparam int  BLANK    = 2;
    localparam int  FLASH    = 30;
    localparam int  N_MODES  = 6;

    // сегменты {g..a} цифр 0..F (как в seg7_dynamic)
    localparam logic [6:0] SEG_REF [16] = '{
        7'h3F, 7'h06, 7'h5B, 7'h4F, 7'h66, 7'h6D, 7'h7D, 7'h07,
        7'h7F, 7'h6F, 7'h77, 7'h7C, 7'h39, 7'h5E, 7'h79, 7'h71
    };
    localparam logic [7:0] LETTER [N_MODES] = '{8'h00, 8'h58, 8'h6D, 8'h78, 8'h50, 8'h79};   // -, c, S, t, r, E
    localparam string      NAME   [N_MODES] = '{"D", "c", "S", "t", "r", "E"};

    class IndModel;
        int          cnt[N_MODES];          // [3] - t, [4] - r, [5] - E
        logic [15:0] last_data = '0;
        logic [7:0]  last_cmd = '0, last_status = '0;
        int          mode = 0;
        bit          err_seen = 0;
        int          tx_flash = 0, rx_flash = 0;

        function new();
            foreach (cnt[i]) cnt[i] = 0;
        endfunction

        // один такт: входы, защёлкнутые DUT по фронту
        function void step(logic tx_frame, logic rx_frame, logic err,
                           logic cmd_we, logic [7:0] info_cmd,
                           logic status_we, logic [7:0] info_status,
                           logic data_we, logic [15:0] info_data,
                           logic clear, logic next_mode);
            if (next_mode) mode = (mode + 1) % N_MODES;
            if (clear) begin
                foreach (cnt[i]) cnt[i] = 0;
                last_data = '0; last_cmd = '0; last_status = '0;
                err_seen = 0;
            end else begin
                if (data_we) last_data = info_data;
                if (cmd_we)    last_cmd    = info_cmd;
                if (status_we) last_status = info_status;
                if (tx_frame) cnt[3] = (cnt[3] + 1) % 1000;
                if (rx_frame) cnt[4] = (cnt[4] + 1) % 1000;
                if (err) begin
                    cnt[5]   = (cnt[5] + 1) % 1000;
                    err_seen = 1;
                end
            end
            tx_flash = tx_frame ? FLASH : (tx_flash > 0 ? tx_flash - 1 : 0);
            rx_flash = rx_frame ? FLASH : (rx_flash > 0 ? rx_flash - 1 : 0);
        endfunction

        // ожидаемые сегменты разрядов (active 1), lit[i] = 0 - разряд погашен
        function void frame(output logic [7:0] f[4], output logic [3:0] lit);
            int v;
            lit = 4'b1111;
            case (mode)
                0: begin
                    for (int i = 0; i < 4; i++) f[i] = {1'b0, SEG_REF[(last_data >> (4 * i)) & 4'hF]};
                end
                1, 2: begin
                    logic [7:0] b = (mode == 1) ? last_cmd : last_status;
                    f[3]   = LETTER[mode];
                    lit[2] = 1'b0;
                    f[2]   = '0;
                    f[1]   = {1'b0, SEG_REF[b[7:4]]};
                    f[0]   = {1'b0, SEG_REF[b[3:0]]};
                end
                default: begin
                    v    = cnt[mode];
                    f[3] = LETTER[mode];
                    f[2] = {1'b0, SEG_REF[(v / 100) % 10]};
                    f[1] = {1'b0, SEG_REF[(v / 10) % 10]};
                    f[0] = {1'b0, SEG_REF[v % 10]};
                end
            endcase
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic        clk   = 1'b0;
    logic        rst_n = 1'b0;
    logic        tx_frame = 0, rx_frame = 0, err = 0;
    logic        cmd_we = 0, status_we = 0, data_we = 0;
    logic [7:0]  info_cmd = '0, info_status = '0;
    logic [15:0] info_data = '0;
    logic        running = 0, clear = 0, next_mode = 0;
    logic [7:0]  seg;
    logic [3:0]  dig;
    logic [3:0]  led;

    always #(CLK_NS / 2) clk = ~clk;

    spi_ind #(
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

    // светодиоды - каждый такт
    always @(posedge clk) begin
        if (rst_n) begin
            automatic logic [3:0] exp = {model.err_seen, running, model.rx_flash != 0, model.tx_flash != 0};
            if (led_on !== exp) fail($sformatf("led = %b, expected %b", led_on, exp));
            model.step(tx_frame, rx_frame, err, cmd_we, info_cmd, status_we, info_status,
                       data_we, info_data, clear, next_mode);
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
        repeat (5) @(posedge clk);
        model.frame(e, e_lit);
        read_display(f, lit);
        if (lit !== e_lit) fail($sformatf("mode %s: lit digits %b, expected %b", NAME[model.mode], lit, e_lit));
        for (int i = 0; i < 4; i++)
            if (e_lit[i] && f[i] !== e[i])
                fail($sformatf("mode %s: digit %0d = %h, expected %h", NAME[model.mode], i, f[i], e[i]));
        $display("[%0t]    mode %s: %h %h %h %h (dig3..dig0)", $time, NAME[model.mode],
                 f[3], f[2], f[1], f[0]);
    endtask

    task automatic pulse_next_mode();
        next_mode <= 1'b1;
        @(posedge clk);
        next_mode <= 1'b0;
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        $display("[%0t] 1. Reset state", $time);
        check_display();

        $display("[%0t] 2. 3000 clocks of random events", $time);
        repeat (3000) begin
            tx_frame    <= ($urandom_range(0, 9) == 0);
            rx_frame    <= ($urandom_range(0, 9) == 0);
            err         <= ($urandom_range(0, 19) == 0);
            cmd_we      <= ($urandom_range(0, 14) == 0);
            info_cmd    <= $urandom;
            status_we   <= ($urandom_range(0, 14) == 0);
            info_status <= $urandom;
            data_we     <= ($urandom_range(0, 14) == 0);
            info_data   <= $urandom;
            running     <= ($urandom_range(0, 99) == 0) ? ~running : running;
            @(posedge clk);
        end
        {tx_frame, rx_frame, err, cmd_we, status_we, data_we} <= '0;

        $display("[%0t] 3. All display modes", $time);
        repeat (N_MODES) begin
            check_display();
            pulse_next_mode();
        end
        check_display();                        // снова режим 0

        $display("[%0t] 4. Counter 999 -> 000", $time);
        repeat (3) pulse_next_mode();           // режим t
        tx_frame <= 1'b1;
        repeat ((998 - model.cnt[3] + 1000) % 1000) @(posedge clk);
        tx_frame <= 1'b0;
        check_display();                        // t998
        repeat (2) begin
            tx_frame <= 1'b1;
            @(posedge clk);
            tx_frame <= 1'b0;
            check_display();                    // t999, t000
        end

        $display("[%0t] 5. clear", $time);
        if (!model.err_seen) fail("no error was generated before clear");
        clear <= 1'b1;
        @(posedge clk);
        clear <= 1'b0;
        repeat (3) @(posedge clk);
        if (led_on[3] !== 1'b0) fail("led[3] is still on after clear");
        repeat (N_MODES) begin
            pulse_next_mode();
            check_display();
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
