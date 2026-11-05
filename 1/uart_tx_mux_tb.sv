// =============================================================================
// Тестбенч для uart_tx_mux (3 источника -> 1 передатчик)
//
// Классы:
//   SeqGen  - генератор последовательностей для источника: длина 1..10 байт,
//             байт = {номер источника[1:0], номер байта в источнике[5:0]};
//   Checker - монитор выхода: последовательности не перемешиваются, байты
//             каждого источника приходят по порядку и все, при выборе нового
//             источника нет готового источника с меньшим номером (приоритет).
// Источники - процессы тестбенча: держат valid, пока байт не принят,
// и делают случайные паузы между последовательностями.
// Проверки:
//   1. После сброса out_valid = 0
//   2. Один источник за раз (каждый из трёх)
//   3. Все источники одновременно, out_ready = 1
//   4. Все источники одновременно, случайный out_ready
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module uart_tx_mux_tb;

    localparam real CLK_NS = 20.0;
    localparam int  N      = 3;

    class SeqGen;
        int         src;
        logic [5:0] cnt = '0;

        function new(int s);
            src = s;
        endfunction

        // новая последовательность: байты и признаки last
        function void make(ref logic [7:0] data[$], ref logic last[$]);
            int len = $urandom_range(1, 10);
            for (int i = 0; i < len; i++) begin
                data.push_back({2'(src), cnt});
                last.push_back(i == len - 1);
                cnt++;
            end
        endfunction
    endclass

    class Checker;
        logic [5:0] next_cnt[N];        // ожидаемый номер байта каждого источника
        int         owner    = -1;      // источник текущей последовательности, -1 - свободен
        int         n_bytes  = 0;
        int         n_seq[N] = '{0, 0, 0};
        int         errors   = 0;

        function new();
            foreach (next_cnt[i]) next_cnt[i] = '0;
        endfunction

        // valid_before - in_valid источников в такте приёма байта
        function void check(logic [7:0] data, logic last, logic [N-1:0] valid_before);
            int src = data[7:6];
            n_bytes++;
            if (owner == -1) begin
                for (int i = 0; i < src; i++)
                    if (valid_before[i])
                        error($sformatf("source %0d chosen while source %0d was ready", src, i));
                owner = src;
            end else if (src != owner) begin
                error($sformatf("byte of source %0d inside sequence of source %0d", src, owner));
            end
            if (data[5:0] !== next_cnt[src])
                error($sformatf("source %0d: byte %0d, expected %0d", src, data[5:0], next_cnt[src]));
            next_cnt[src] = data[5:0] + 1'b1;
            if (last) begin
                owner = -1;
                n_seq[src]++;
            end
        endfunction

        function void error(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic                clk   = 1'b0;
    logic                rst_n = 1'b0;
    logic [N-1:0][7:0]   in_data;
    logic [N-1:0]        in_valid;
    logic [N-1:0]        in_last;
    logic [N-1:0]        in_ready;
    logic [7:0]          out_data;
    logic                out_valid;
    logic                out_ready = 1'b1;
    logic                out_last;

    always #(CLK_NS / 2) clk = ~clk;

    uart_tx_mux #(.N(N)) dut (.*);

    Checker      chk = new();
    SeqGen       gen[N];
    logic [7:0]  q_data[N][$];          // очереди байт источников
    logic        q_last[N][$];
    int          sent[N]   = '{0, 0, 0};
    bit          enable[N] = '{0, 0, 0};    // источник выдаёт новые последовательности
    int          ready_pct = 100;

    // -------------------------------------------------------------------------
    // Источники: valid = есть байт в очереди; выходы обновляются по такту
    // -------------------------------------------------------------------------
    initial begin
        in_valid = '0;
        in_data  = '0;
        in_last  = '0;
    end

    always @(posedge clk) begin
        if (rst_n) begin
            if (out_valid && out_ready) chk.check(out_data, out_last, in_valid);
            if (out_valid !== |in_valid) chk.error("out_valid != OR of in_valid");
            if ($countones(in_ready) > 1) chk.error("several in_ready at once");

            for (int i = 0; i < N; i++) begin
                if (in_valid[i] && in_ready[i]) begin
                    void'(q_data[i].pop_front());
                    void'(q_last[i].pop_front());
                end
                // очередь пуста: с вероятностью 1/4 новая последовательность
                if (enable[i] && q_data[i].size() == 0 && $urandom_range(0, 3) == 0) begin
                    gen[i].make(q_data[i], q_last[i]);
                    sent[i]++;
                end
                in_valid[i] <= (q_data[i].size() != 0);
                in_data[i]  <= (q_data[i].size() != 0) ? q_data[i][0] : 8'h00;
                in_last[i]  <= (q_data[i].size() != 0) ? q_last[i][0] : 1'b0;
            end
        end
        out_ready <= ($urandom_range(1, 100) <= ready_pct);
    end

    task automatic run_phase(input bit [N-1:0] en, input int clks);
        foreach (enable[i]) enable[i] = en[i];
        repeat (clks) @(posedge clk);
        foreach (enable[i]) enable[i] = 0;
        // дождаться, пока все очереди опустеют
        while (q_data[0].size() || q_data[1].size() || q_data[2].size() || in_valid) @(posedge clk);
        repeat (3) @(posedge clk);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        foreach (gen[i]) gen[i] = new(i);
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        $display("[%0t] 1. Reset state", $time);
        if (out_valid !== 1'b0) chk.error("after reset out_valid != 0");

        $display("[%0t] 2. One source at a time", $time);
        run_phase(3'b001, 300);
        run_phase(3'b010, 300);
        run_phase(3'b100, 300);

        $display("[%0t] 3. All sources, out_ready = 1", $time);
        run_phase(3'b111, 3000);

        $display("[%0t] 4. All sources, random out_ready", $time);
        ready_pct = 40;
        run_phase(3'b111, 6000);

        foreach (sent[i])
            if (chk.n_seq[i] != sent[i])
                chk.error($sformatf("source %0d: sent %0d sequences, received %0d", i, sent[i], chk.n_seq[i]));

        $display("------------------------------------------------------------");
        $display("sequences: src0 %0d, src1 %0d, src2 %0d; bytes %0d; ERRORS: %0d",
                 chk.n_seq[0], chk.n_seq[1], chk.n_seq[2], chk.n_bytes, chk.errors);
        if (chk.errors == 0) $display("TEST PASSED");
        else                 $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #10ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
