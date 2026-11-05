// =============================================================================
// Тестбенч для temp_gen: генератор последовательности температур
//
// Класс RefGen - эталон: те же правила в целых числах (шаг 0.125 °C, 11 бит со
// знаком), плюс собственный LFSR. Тестбенч подаёт импульсы tick и после каждого
// сравнивает temp_val и temp_word с эталоном.
// Проверки:
//   1. После сброса: temp_val = 0
//   2. MODE_CONST: после тиков значение gen_lo, в том числе отрицательное
//   3. MODE_LINEAR: рост с шагом до gen_hi и остановка на gen_hi
//   4. MODE_SAW: рост и возврат в gen_lo
//   5. MODE_NOISE: значения совпадают с эталоном, лежат в [lo, min(hi, lo + mask)],
//      значений достаточно разных
//   6. Смена режима и restart начинают последовательность заново; без tick значение
//      не меняется
//   7. Граничные значения: lo = -128 °C, hi = +127.875 °C, шаг 2047 (без переполнения 11 бит)
//   8. 300 случайных режимов и параметров против эталона
//   9. temp_word = {temp_val, 5'b0}
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module temp_gen_tb;

    localparam real CLK_NS = 20.0;

    localparam logic [15:0] SEED = 16'hACE1;

    class RefGen;
        int unsigned lfsr = SEED;
        int          val  = 0;

        function void restart(int lo);
            val  = lo;
            lfsr = SEED;
        endfunction

        function void tick(int mode, int lo, int hi, int step, int mask);
            int nxt = val + step;
            int rnd = lfsr & mask & 8'hFF;
            int fb  = ((lfsr >> 15) ^ (lfsr >> 14) ^ (lfsr >> 12) ^ (lfsr >> 3)) & 1;
            lfsr = ((lfsr << 1) | fb) & 16'hFFFF;
            case (mode)
                0: val = lo;
                1: val = (nxt > hi) ? hi : nxt;
                2: val = (nxt > hi) ? lo : nxt;
                default: val = (lo + rnd > hi) ? hi : lo + rnd;
            endcase
        endfunction
    endclass

    logic               clk = 1'b0, rst_n = 1'b0;
    logic               tick = 1'b0, restart = 1'b0;
    logic        [1:0]  gen_mode = '0;
    logic signed [10:0] gen_lo = '0, gen_hi = '0;
    logic        [10:0] gen_step = 11'd1;
    logic        [7:0]  gen_noise_mask = '0;
    logic signed [10:0] temp_val;
    logic        [15:0] temp_word;

    int errors = 0;

    always #(CLK_NS / 2) clk = ~clk;

    temp_gen #(.LFSR_SEED (SEED)) dut (.*);

    RefGen ref_g;

    task automatic err(string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    task automatic check(string name);
        if (int'(temp_val) !== ref_g.val)
            err($sformatf("%s: temp_val = %0d, expected %0d", name, int'(temp_val), ref_g.val));
        if (temp_word !== {temp_val, 5'b0})
            err($sformatf("%s: temp_word = %h, expected %h", name, temp_word, {temp_val, 5'b0}));
    endtask

    // выставить параметры (без смены режима)
    task automatic set_params(int lo, int hi, int step, int mask);
        @(negedge clk);
        gen_lo = lo;  gen_hi = hi;  gen_step = step;  gen_noise_mask = mask;
    endtask

    task automatic do_restart();
        @(negedge clk);
        restart = 1'b1;
        @(negedge clk);
        restart = 1'b0;
        @(posedge clk);
        #1;
        ref_g.restart(int'(gen_lo));
    endtask

    // сменить режим: последовательность начинается заново (тот же режим - restart)
    task automatic set_mode(int mode);
        if (int'(gen_mode) == mode) begin
            do_restart();
        end else begin
            @(negedge clk);
            gen_mode = mode;
            repeat (3) @(posedge clk);
            #1;
            ref_g.restart(int'(gen_lo));
        end
    endtask

    // n тиков с проверкой после каждого
    task automatic run_ticks(string name, int n);
        for (int i = 0; i < n; i++) begin
            @(negedge clk);
            tick = 1'b1;
            @(negedge clk);
            tick = 1'b0;
            ref_g.tick(int'(gen_mode), int'(gen_lo), int'(gen_hi), int'(gen_step), int'(gen_noise_mask));
            @(posedge clk);
            #1;
            check($sformatf("%s tick %0d", name, i));
        end
    endtask

    initial begin
        int  seen [int];
        int  mn, mx, t_prev;
        ref_g = new();

        // 1. сброс
        #(CLK_NS * 3);
        if (temp_val !== 11'sd0) err($sformatf("after reset temp_val = %0d", temp_val));
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        // 2. константа
        set_params(203, 400, 1, 0);          // 25.375 °C
        do_restart();
        check("const 25.375 start");
        if (temp_word !== 16'h1960) err($sformatf("25.375 degC = %h, expected 1960", temp_word));
        run_ticks("const", 5);
        set_params(-8, 100, 1, 0);           // -1.0 °C
        do_restart();
        run_ticks("const neg", 3);
        if (temp_word !== 16'hFF00) err($sformatf("-1.0 degC = %h, expected FF00", temp_word));

        // 3. линейный рост
        set_params(-8, 20, 3, 0);
        set_mode(1);
        check("linear start");
        run_ticks("linear", 14);
        if (int'(temp_val) !== 20) err($sformatf("linear did not stop at hi: %0d", int'(temp_val)));

        // 4. пила
        set_params(-8, 20, 5, 0);
        set_mode(2);
        check("saw start");
        run_ticks("saw", 20);

        // 5. шум
        set_params(100, 400, 1, 8'h0F);
        set_mode(3);
        check("noise start");
        mn = 10000;  mx = -10000;
        for (int i = 0; i < 400; i++) begin
            run_ticks("noise", 1);
            seen[int'(temp_val)] = 1;
            if (int'(temp_val) < mn) mn = int'(temp_val);
            if (int'(temp_val) > mx) mx = int'(temp_val);
        end
        if (mn < 100 || mx > 115) err($sformatf("noise range %0d..%0d, expected 100..115", mn, mx));
        if (seen.size() < 12)     err($sformatf("noise: only %0d different values", seen.size()));
        // шум с ограничением сверху
        set_params(100, 103, 1, 8'hFF);
        do_restart();
        run_ticks("noise clamp", 100);
        if (int'(temp_val) > 103) err("noise exceeded gen_hi");

        // 6. смена режима и restart, значение без tick не меняется
        set_params(-8, 100, 4, 0);
        set_mode(1);
        run_ticks("linear2", 6);
        t_prev = int'(temp_val);
        repeat (50) @(posedge clk);
        #1;
        if (int'(temp_val) !== t_prev) err("temp_val changed without tick");
        do_restart();
        check("restart");
        if (int'(temp_val) !== -8) err("restart did not return to gen_lo");
        run_ticks("after restart", 4);
        set_mode(2);
        if (int'(temp_val) !== -8) err("mode change did not restart");
        check("mode change");

        // 7. границы 11 бит
        set_params(-128, 127, 100, 0);
        set_mode(1);
        run_ticks("limits linear", 6);
        if (int'(temp_val) !== 127) err($sformatf("limit linear = %0d, expected 127", int'(temp_val)));
        set_mode(2);
        run_ticks("limits saw", 6);
        set_params(-128, 127, 1023, 0);        // шаг больше всего диапазона
        do_restart();
        run_ticks("limits big step", 4);
        set_params(127, 127, 1, 8'hFF);
        set_mode(3);
        run_ticks("limits noise", 10);

        // 7а. полный диапазон 11 бит: -128 .. +127.875 °C
        set_params(-1024, 1023, 2047, 0);
        set_mode(1);
        run_ticks("full linear", 4);
        if (int'(temp_val) !== 1023) err("full range linear: not at 1023");
        set_mode(2);
        run_ticks("full saw", 4);
        set_params(-1024, 1023, 300, 8'hFF);
        set_mode(3);
        run_ticks("full noise", 20);
        set_mode(2);
        run_ticks("full saw 300", 15);

        // 8. случайные режимы и параметры
        for (int k = 0; k < 300; k++) begin
            int lo, hi, mode;
            lo   = $urandom_range(0, 2047) - 1024;
            hi   = lo + $urandom_range(0, 1023 - ((lo > 0) ? lo : 0));
            mode = $urandom_range(0, 3);
            if (hi > 1023) hi = 1023;
            set_params(lo, hi, $urandom_range(1, 200), $urandom_range(0, 255));
            set_mode(mode);
            run_ticks($sformatf("rand%0d mode %0d", k, mode), $urandom_range(1, 12));
        end

        repeat (5) @(posedge clk);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED: %0d errors", errors);
        $finish;
    end

endmodule
