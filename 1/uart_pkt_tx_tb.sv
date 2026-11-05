// =============================================================================
// Тестбенч для uart_pkt_tx (формирователь пакета МО -> МУ)
//
// Классы:
//   Packet     - пакет: поля MEAS, FILT, ERR, байты на линии (с CHK);
//   Scoreboard - очередь ожидаемых байт и признаков last, сравнение.
// Проверки:
//   1. После сброса: in_ready = 1, tx_valid = 0
//   2. Граничные пакеты (0x0000, 0xFFFF, 0xAA внутри данных), выход всегда готов
//   3. Случайные пакеты, случайные паузы на входе и случайный tx_ready
//   На всём протяжении:
//   - байты, порядок и CHK совпадают с эталоном, tx_last только на 8-м байте;
//   - пока tx_valid = 1 и tx_ready = 0, tx_data и tx_last не меняются;
//   - пока пакет передаётся, in_ready = 0.
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module uart_pkt_tx_tb;

    localparam real CLK_NS = 20.0;
    localparam int  N_RAND = 200;

    // -------------------------------------------------------------------------
    // Классы
    // -------------------------------------------------------------------------
    class Packet;
        logic [15:0] meas, filt, err;

        function new(logic [15:0] m = $urandom, logic [15:0] f = $urandom,
                     logic [15:0] e = $urandom);
            meas = m;
            filt = f;
            err  = e;
        endfunction

        // байты пакета на линии, CHK = XOR байтов 1..6
        function void to_bytes(ref logic [7:0] b[$]);
            logic [7:0] d[6];
            logic [7:0] chk = '0;
            d = '{meas[15:8], meas[7:0], filt[15:8], filt[7:0], err[15:8], err[7:0]};
            b.push_back(8'hAA);
            foreach (d[i]) begin
                b.push_back(d[i]);
                chk ^= d[i];
            end
            b.push_back(chk);
        endfunction
    endclass

    class Scoreboard;
        logic [7:0] exp_data[$];
        int         exp_pos[$];     // номер байта в пакете (0..7)
        int         n_bytes = 0;
        int         n_pkts  = 0;
        int         errors  = 0;

        function void expect_pkt(Packet p);
            logic [7:0] b[$];
            p.to_bytes(b);
            foreach (b[i]) begin
                exp_data.push_back(b[i]);
                exp_pos.push_back(i);
            end
        endfunction

        function void check(logic [7:0] data, logic last);
            logic [7:0] d;
            int         pos;
            if (exp_data.size() == 0) begin
                error($sformatf("unexpected byte %h", data));
                return;
            end
            d   = exp_data.pop_front();
            pos = exp_pos.pop_front();
            n_bytes++;
            if (data !== d)
                error($sformatf("byte %0d: got %h, expected %h", pos, data, d));
            if (last !== (pos == 7))
                error($sformatf("byte %0d: tx_last = %b", pos, last));
            if (pos == 7) n_pkts++;
        endfunction

        function void error(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic        clk      = 1'b0;
    logic        rst_n    = 1'b0;
    logic [15:0] meas     = '0;
    logic [15:0] filt     = '0;
    logic [15:0] err      = '0;
    logic        in_valid = 1'b0;
    logic        in_ready;
    logic [7:0]  tx_data;
    logic        tx_valid;
    logic        tx_ready = 1'b0;
    logic        tx_last;

    always #(CLK_NS / 2) clk = ~clk;

    uart_pkt_tx dut (.*);

    Scoreboard sb = new();
    int        ready_pct = 100;     // вероятность tx_ready = 1, %

    // -------------------------------------------------------------------------
    // Приёмник байт (монитор + случайный tx_ready)
    // -------------------------------------------------------------------------
    logic [7:0] hold_data;
    logic       hold_last;
    logic       holding = 1'b0;     // в прошлом такте был valid без ready

    always @(posedge clk) begin
        if (rst_n) begin
            if (holding && (!tx_valid || tx_data !== hold_data || tx_last !== hold_last))
                sb.error("tx_data/tx_last/tx_valid changed while waiting for tx_ready");
            if (tx_valid && in_ready)
                sb.error("in_ready = 1 while packet is being sent");

            if (tx_valid && tx_ready) sb.check(tx_data, tx_last);

            holding   <= tx_valid && !tx_ready;
            hold_data <= tx_data;
            hold_last <= tx_last;
        end
        tx_ready <= ($urandom_range(1, 100) <= ready_pct);
    end

    // -------------------------------------------------------------------------
    // Драйвер входа
    // -------------------------------------------------------------------------
    task automatic send(Packet p, int max_gap = 0);
        repeat ($urandom_range(0, max_gap)) @(posedge clk);
        meas     <= p.meas;
        filt     <= p.filt;
        err      <= p.err;
        in_valid <= 1'b1;
        @(posedge clk);
        while (!in_ready) @(posedge clk);
        sb.expect_pkt(p);
        in_valid <= 1'b0;
        meas     <= 'x;                 // после приёма вход может меняться
        filt     <= 'x;
        err      <= 'x;
    endtask

    task automatic drain();
        int t = 0;
        while ((sb.exp_data.size() != 0 || tx_valid) && t < 100_000) begin
            @(posedge clk);
            t++;
        end
        if (sb.exp_data.size() != 0) sb.error("timeout: not all bytes were sent");
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        Packet p;

        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        // 1 ------------------------------------------------------------------
        $display("[%0t] 1. Reset state", $time);
        if (in_ready !== 1'b1) sb.error("after reset in_ready != 1");
        if (tx_valid !== 1'b0) sb.error("after reset tx_valid != 0");

        // 2 ------------------------------------------------------------------
        $display("[%0t] 2. Corner packets, tx_ready = 1", $time);
        ready_pct = 100;
        p = new(16'h0000, 16'h0000, 16'h0000); send(p);
        p = new(16'hFFFF, 16'hFFFF, 16'hFFFF); send(p);
        p = new(16'hAAAA, 16'h00AA, 16'hAA00); send(p);
        p = new(16'h1960, 16'h1900, 16'h0060); send(p);   // пример из PROTOCOL.md
        drain();

        // 3 ------------------------------------------------------------------
        $display("[%0t] 3. %0d random packets, random gaps and tx_ready", $time, N_RAND);
        ready_pct = 30;
        for (int i = 0; i < N_RAND; i++) begin
            p = new();
            send(p, (i % 2) ? 5 : 0);
        end
        drain();

        // Итог ---------------------------------------------------------------
        $display("------------------------------------------------------------");
        $display("packets: %0d, bytes: %0d, ERRORS: %0d", sb.n_pkts, sb.n_bytes, sb.errors);
        if (sb.errors == 0 && sb.n_pkts == N_RAND + 4) $display("TEST PASSED");
        else                                           $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #10ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
