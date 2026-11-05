// =============================================================================
// Тестбенч для uart_seq_tx (отправка заданной последовательности байт)
//
// Два экземпляра DUT:
//   dut_str - строка "UART_top2\r\n" (11 байт), как в UART_top2;
//   dut_one - последовательность из одного байта (крайний случай LEN = 1).
// Проверки:
//   1. После сброса: busy = 0, tx_valid = 0
//   2. Последовательность целиком, по порядку, tx_last только на последнем байте
//   3. start во время передачи игнорируется: принимается только start при busy = 0,
//      число последовательностей на выходе = числу принятых start
//   4. Случайный tx_ready: пока ждём ready, tx_data / tx_last не меняются
//   5. Многократный запуск подряд (start сразу после окончания)
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module uart_seq_tx_tb;

    localparam real CLK_NS = 20.0;
    localparam int  LEN    = 11;
    localparam logic [8*LEN-1:0] SEQ = {"UART_top2", 8'h0D, 8'h0A};

    // -------------------------------------------------------------------------
    // Монитор одного выхода: сравнивает поток с повторениями эталона
    // -------------------------------------------------------------------------
    class SeqMonitor;
        string      name;
        logic [7:0] ref_bytes[$];
        int         pos    = 0;     // номер ожидаемого байта в последовательности
        int         n_seq  = 0;     // принято последовательностей целиком
        int         errors = 0;

        function new(string n, logic [7:0] r[$]);
            name      = n;
            ref_bytes = r;
        endfunction

        function void check(logic [7:0] data, logic last);
            if (data !== ref_bytes[pos])
                error($sformatf("byte %0d: got %h, expected %h", pos, data, ref_bytes[pos]));
            if (last !== (pos == ref_bytes.size() - 1))
                error($sformatf("byte %0d: tx_last = %b", pos, last));
            pos++;
            if (pos == ref_bytes.size()) begin
                pos = 0;
                n_seq++;
            end
        endfunction

        function void error(string msg);
            errors++;
            if (errors <= 10) $display("[%0t] ERROR %s: %s", $time, name, msg);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk   = 1'b0;
    logic       rst_n = 1'b0;
    logic       start = 1'b0;
    logic       tx_ready = 1'b1;
    logic       busy_s, valid_s, last_s, busy_1, valid_1, last_1;
    logic [7:0] data_s, data_1;

    always #(CLK_NS / 2) clk = ~clk;

    uart_seq_tx #(.LEN(LEN), .SEQ(SEQ)) dut_str (
        .clk (clk), .rst_n (rst_n), .start (start), .busy (busy_s),
        .tx_data (data_s), .tx_valid (valid_s), .tx_ready (tx_ready), .tx_last (last_s)
    );

    uart_seq_tx #(.LEN(1), .SEQ(8'h5A)) dut_one (
        .clk (clk), .rst_n (rst_n), .start (start), .busy (busy_1),
        .tx_data (data_1), .tx_valid (valid_1), .tx_ready (tx_ready), .tx_last (last_1)
    );

    SeqMonitor mon_s;
    SeqMonitor mon_1;
    int        ready_pct = 100;
    int        errors    = 0;
    int        exp_s     = 0;       // сколько start должно быть принято (DUT свободен)
    int        exp_1     = 0;

    initial begin
        logic [7:0] r[$];
        for (int i = LEN - 1; i >= 0; i--) r.push_back(SEQ[8*i +: 8]);
        mon_s = new("str", r);
        mon_1 = new("one", '{8'h5A});
    end

    // монитор + случайный tx_ready
    logic [8:0] hold_s, hold_1;         // {last, data} в прошлом такте
    logic       wait_s = 0, wait_1 = 0; // в прошлом такте valid без ready

    always @(posedge clk) begin
        if (rst_n) begin
            if (wait_s && (!valid_s || {last_s, data_s} !== hold_s)) mon_s.error("output changed before ready");
            if (wait_1 && (!valid_1 || {last_1, data_1} !== hold_1)) mon_1.error("output changed before ready");
            if (valid_s !== busy_s) mon_s.error("tx_valid != busy");
            if (valid_s && tx_ready) mon_s.check(data_s, last_s);
            if (valid_1 && tx_ready) mon_1.check(data_1, last_1);
            if (start && !busy_s) exp_s++;
            if (start && !busy_1) exp_1++;
        end
        wait_s   <= valid_s && !tx_ready;
        wait_1   <= valid_1 && !tx_ready;
        hold_s   <= {last_s, data_s};
        hold_1   <= {last_1, data_1};
        tx_ready <= ($urandom_range(1, 100) <= ready_pct);
    end

    task automatic pulse_start();
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;
    endtask

    task automatic wait_idle();
        @(posedge clk);
        while (busy_s || busy_1) @(posedge clk);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        $display("[%0t] 1. Reset state", $time);
        if (busy_s !== 1'b0 || valid_s !== 1'b0) mon_s.error("after reset busy/tx_valid != 0");
        if (busy_1 !== 1'b0 || valid_1 !== 1'b0) mon_1.error("after reset busy/tx_valid != 0");

        $display("[%0t] 2. One sequence, tx_ready = 1", $time);
        pulse_start();
        wait_idle();

        $display("[%0t] 3. start while busy is ignored", $time);
        ready_pct = 20;
        pulse_start();
        repeat (3) begin
            repeat (5) @(posedge clk);
            pulse_start();
        end
        wait_idle();

        $display("[%0t] 4-5. 20 sequences, random tx_ready, start right after the end", $time);
        ready_pct = 50;
        repeat (20) begin
            pulse_start();
            wait_idle();
        end
        repeat (5) @(posedge clk);

        errors = mon_s.errors + mon_1.errors;
        if (mon_s.n_seq != exp_s || mon_s.pos != 0 || exp_s != 22) begin
            errors++;
            $display("ERROR str: %0d sequences (+%0d bytes), expected %0d (22)", mon_s.n_seq, mon_s.pos, exp_s);
        end
        if (mon_1.n_seq != exp_1) begin
            errors++;
            $display("ERROR one: %0d sequences, expected %0d", mon_1.n_seq, exp_1);
        end

        $display("------------------------------------------------------------");
        $display("sequences: str %0d of %0d, one %0d of %0d, ERRORS: %0d", mon_s.n_seq, exp_s, mon_1.n_seq, exp_1, errors);
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
