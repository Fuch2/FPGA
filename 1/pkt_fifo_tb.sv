// =============================================================================
// Тестбенч для pkt_fifo (очередь слов, valid/ready)
//
// Классы:
//   FifoModel - эталонная очередь: слова в порядке записи, проверка чтения.
// Проверки:
//   1. После сброса: пусто, in_ready = 1, count = 0
//   2. Заполнение до DEPTH: in_ready = 0, лишняя запись не принимается
//   3. Опустошение: слова по порядку, out_valid = 0 в конце
//   4. 20000 тактов случайных in_valid / out_ready (запись и чтение в одном такте)
//   Каждый такт: count и флаги совпадают с моделью, out_data держится,
//   пока ждём out_ready.
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module pkt_fifo_tb;

    localparam real CLK_NS = 20.0;
    localparam int  WIDTH  = 48;
    localparam int  DEPTH  = 4;

    class FifoModel;
        logic [WIDTH-1:0] q[$];
        int               n_wr = 0, n_rd = 0, errors = 0;

        function void error(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    logic             clk       = 1'b0;
    logic             rst_n     = 1'b0;
    logic [WIDTH-1:0] in_data   = '0;
    logic             in_valid  = 1'b0;
    logic             in_ready;
    logic [WIDTH-1:0] out_data;
    logic             out_valid;
    logic             out_ready = 1'b0;
    logic [$clog2(DEPTH):0] count;

    always #(CLK_NS / 2) clk = ~clk;

    pkt_fifo #(.WIDTH(WIDTH), .DEPTH(DEPTH)) dut (.*);

    FifoModel m = new();
    int wr_pct = 0, rd_pct = 0;     // вероятности in_valid и out_ready, %

    // монитор + случайные входы
    always @(posedge clk) begin
        if (rst_n) begin
            if (count !== m.q.size())             m.error($sformatf("count = %0d, model %0d", count, m.q.size()));
            if (in_ready !== (m.q.size() < DEPTH)) m.error("in_ready does not match fullness");
            if (out_valid !== (m.q.size() != 0))   m.error("out_valid does not match emptiness");
            if (out_valid && out_data !== m.q[0])  m.error($sformatf("out_data = %h, expected %h", out_data, m.q[0]));
            if (out_valid && out_ready) begin
                void'(m.q.pop_front());
                m.n_rd++;
            end
            if (in_valid && in_ready) begin
                m.q.push_back(in_data);
                m.n_wr++;
            end
        end
        in_valid  <= ($urandom_range(1, 100) <= wr_pct);
        in_data   <= {$urandom, $urandom};
        out_ready <= ($urandom_range(1, 100) <= rd_pct);
    end

    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        $display("[%0t] 1. Reset state", $time);
        repeat (2) @(posedge clk);

        $display("[%0t] 2. Fill up, writes into full FIFO are refused", $time);
        wr_pct = 100; rd_pct = 0;
        repeat (3 * DEPTH) @(posedge clk);
        if (m.q.size() != DEPTH) m.error($sformatf("%0d words after filling", m.q.size()));

        $display("[%0t] 3. Drain", $time);
        wr_pct = 0; rd_pct = 100;
        repeat (3 * DEPTH) @(posedge clk);
        if (m.q.size() != 0) m.error("FIFO is not empty after draining");

        $display("[%0t] 4. Random traffic", $time);
        wr_pct = 50; rd_pct = 50;
        repeat (10000) @(posedge clk);
        wr_pct = 70; rd_pct = 40;
        repeat (5000) @(posedge clk);
        wr_pct = 40; rd_pct = 70;
        repeat (5000) @(posedge clk);
        wr_pct = 0; rd_pct = 100;
        repeat (3 * DEPTH) @(posedge clk);

        $display("------------------------------------------------------------");
        $display("written: %0d, read: %0d, ERRORS: %0d", m.n_wr, m.n_rd, m.errors);
        if (m.errors == 0 && m.n_wr == m.n_rd && m.n_wr > 1000) $display("TEST PASSED");
        else                                                   $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #10ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
