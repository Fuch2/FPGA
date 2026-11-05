// =============================================================================
// Тестбенч для test_pkt_gen (генератор тестовых пакетов)
//
// Классы:
//   GenModel - эталон: значения пакета номер n и насыщающее вычитание;
//              монитор сравнивает каждый принятый пакет с эталоном.
// Два экземпляра DUT: dut (25.0 °C, шаг 0.125 °C, как на плате) и
// dut_sat (BASE = 0x7E00, шаг 0x0100 - meas переходит через 0x8000,
// err уходит в насыщение -32768).
// Проверки:
//   1. После сброса valid = 0; при run = 0 пакетов нет
//   2. single при run = 0: по одному пакету n = 0, 1, 2
//   3. restart + run = 1: первый пакет сразу, затем ровно через PERIOD тактов
//   4. ready = 0: valid и данные держатся, пропущенные моменты не копятся,
//      номер n не пропускается
//   5. Насыщение err (dut_sat) и функция sat_sub на 20000 случайных парах
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module test_pkt_gen_tb;

    localparam real CLK_NS = 20.0;
    localparam int  PERIOD = 50;

    class GenModel;
        string       name;
        logic [15:0] base, step;
        int          n       = 0;       // номер следующего ожидаемого пакета
        int          n_pkts  = 0;
        int          errors  = 0;

        function new(string nm, logic [15:0] b, logic [15:0] s);
            name = nm;
            base = b;
            step = s;
        endfunction

        static function logic [15:0] sat(logic [15:0] a, logic [15:0] b);
            int d = int'($signed(a)) - int'($signed(b));
            if (d >  32767) return 16'h7FFF;
            if (d < -32768) return 16'h8000;
            return 16'(d);
        endfunction

        function void check(logic [15:0] meas, logic [15:0] filt, logic [15:0] err);
            logic [15:0] m = base + 16'(n) * step;
            logic [15:0] f = base + 16'(n) * (step >> 1);
            logic [15:0] e = sat(m, f);
            if ({meas, filt, err} !== {m, f, e})
                error($sformatf("packet %0d: %h %h %h, expected %h %h %h",
                                n, meas, filt, err, m, f, e));
            n++;
            n_pkts++;
        endfunction

        function void error(string msg);
            errors++;
            if (errors <= 10) $display("[%0t] ERROR %s: %s", $time, name, msg);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic        clk     = 1'b0;
    logic        rst_n   = 1'b0;
    logic        run     = 1'b0;
    logic        single  = 1'b0;
    logic        restart = 1'b0;
    logic        ready   = 1'b1;
    logic [15:0] meas, filt, err, s_meas, s_filt, s_err;
    logic        valid, s_valid;

    always #(CLK_NS / 2) clk = ~clk;

    test_pkt_gen #(.PERIOD_CLKS(PERIOD)) dut (.*);

    test_pkt_gen #(
        .PERIOD_CLKS (PERIOD),
        .MEAS_BASE   (16'h7E00),
        .MEAS_STEP   (16'h0100)
    ) dut_sat (
        .clk (clk), .rst_n (rst_n), .run (run), .single (single), .restart (restart),
        .meas (s_meas), .filt (s_filt), .err (s_err), .valid (s_valid), .ready (ready)
    );

    GenModel m_main = new("main", 16'h1900, 16'h0020);
    GenModel m_sat  = new("sat",  16'h7E00, 16'h0100);

    // -------------------------------------------------------------------------
    // Монитор
    // -------------------------------------------------------------------------
    logic [47:0] hold;
    logic        waiting = 1'b0;
    longint      cyc = 0, last_pkt_cyc = -1;
    bit          check_period = 0;
    int          sat_seen     = 0;      // пакетов dut_sat с err = -32768

    always @(posedge clk) begin
        cyc++;
        if (rst_n) begin
            if (waiting && (!valid || {meas, filt, err} !== hold))
                m_main.error("valid/data changed while ready = 0");
            if (valid && ready) begin
                m_main.check(meas, filt, err);
                if (check_period && last_pkt_cyc >= 0 && cyc - last_pkt_cyc != PERIOD)
                    m_main.error($sformatf("interval %0d clocks, expected %0d", cyc - last_pkt_cyc, PERIOD));
                last_pkt_cyc = cyc;
            end
            if (s_valid && ready) begin
                m_sat.check(s_meas, s_filt, s_err);
                if (s_err == 16'h8000) sat_seen++;
            end
        end
        waiting <= valid && !ready;
        hold    <= {meas, filt, err};
    end

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        int got;

        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        $display("[%0t] 1. Reset state, run = 0", $time);
        repeat (5 * PERIOD) @(posedge clk);
        if (m_main.n_pkts != 0) m_main.error("packets while run = 0");

        $display("[%0t] 2. single pulses", $time);
        repeat (3) begin
            single <= 1'b1;
            @(posedge clk);
            single <= 1'b0;
            repeat (10) @(posedge clk);
        end
        if (m_main.n_pkts != 3) m_main.error($sformatf("%0d packets after 3 single", m_main.n_pkts));

        $display("[%0t] 3. restart + run: first packet at once, then every %0d clocks", $time, PERIOD);
        m_main.n = 0;
        m_sat.n  = 0;
        restart <= 1'b1;
        run     <= 1'b1;
        @(posedge clk);
        restart <= 1'b0;
        got = m_main.n_pkts;
        repeat (3) @(posedge clk);
        if (m_main.n_pkts != got + 1) m_main.error("no packet right after restart");
        check_period = 1;
        repeat (20 * PERIOD) @(posedge clk);
        check_period = 0;
        if (m_main.n_pkts != got + 21) m_main.error($sformatf("%0d packets in 20 periods", m_main.n_pkts - got - 1));

        $display("[%0t] 4. ready = 0 for 5 periods", $time);
        ready <= 1'b0;
        repeat (5 * PERIOD) @(posedge clk);
        got = m_main.n_pkts;
        ready <= 1'b1;
        repeat (PERIOD + 5) @(posedge clk);
        // после ready: отдан задержанный пакет и ещё не больше одного
        if (m_main.n_pkts - got < 1 || m_main.n_pkts - got > 2)
            m_main.error($sformatf("%0d packets after ready returned", m_main.n_pkts - got));
        run <= 1'b0;
        repeat (2 * PERIOD) @(posedge clk);

        $display("[%0t] 5. Saturation: %0d of %0d dut_sat packets saturated; sat_sub on random pairs",
                 $time, sat_seen, m_sat.n_pkts);
        if (sat_seen == 0) m_sat.error("no packet with saturated err = 8000");
        for (int i = 0; i < 20000; i++) begin
            automatic logic [15:0] a = $urandom, b = $urandom;
            if (i < 4) begin                        // крайние значения
                a = (i[0]) ? 16'h7FFF : 16'h8000;
                b = (i[0]) ? 16'h8000 : 16'h7FFF;
            end
            if (dut.sat_sub(a, b) !== GenModel::sat(a, b))
                m_main.error($sformatf("sat_sub(%h, %h) = %h, expected %h",
                                       a, b, dut.sat_sub(a, b), GenModel::sat(a, b)));
        end

        $display("------------------------------------------------------------");
        $display("packets: main %0d, sat %0d, ERRORS: %0d", m_main.n_pkts, m_sat.n_pkts,
                 m_main.errors + m_sat.errors);
        if (m_main.errors + m_sat.errors == 0) $display("TEST PASSED");
        else                                   $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #10ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
