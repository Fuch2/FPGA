// =============================================================================
// Тестбенч для i2c_seq (конвейер: сценарии транзакций) и i2c_res_parser (разбор результата)
//
// Цепочка: i2c_seq -> i2c_master -> шина с двумя моделями ведомых -> i2c_res_parser.
// «Датчик» 0x48: температура 0x1960 (байты 0 и 1), Tos 0x5000 (байты 3 и 4).
// «EEPROM» 0x50..0x53: страница 16 байт, цикл записи 5 мс (в это время NACK на адрес).
// Классы:
//   Counters - счётчики импульсов парсера txn_p / nack_p / err_p и событий шины;
//   Check    - счётчик ошибок и сравнения.
// Проверки:
//   1. SCAN: список найденных {0x48, 0x50, 0x51, 0x52, 0x53}, 112 транзакций, 107 NACK,
//      ошибок нет; повторный SCAN не дублирует список
//   2. DT_TEMP: rd_word = 0x1960, повторный START (2 START, 1 STOP), статус «чтение»
//   3. DT_TOS: rd_word = 0x5000
//   4. EE_READ: 4 байта с адреса 0 -> rd_word = последние два байта
//   5. EE_WRITE дважды: значение PATTERN_BASE + номер запуска записано в ячейку EEPROM,
//      прочитано обратно; опрос ACK: NACK пока идёт цикл записи, затем ACK; ошибок нет
//   6. ABSENT: NACK ожидаем, ошибка не засчитывается, статус 0x01
//   7. Датчик отсутствует: сценарий прерван, seq.error = 1, ошибка засчитана, данных нет
//   8. EEPROM не принимает байт данных: NACK данных - ошибка, сценарий прерван
//   9. Датчик растягивает SCL дольше тайм-аута: timeout - ошибка, сценарий прерван
//  10. EEPROM пропала во время опроса ACK: после POLL_MAX опросов ошибка
//  11. start во время работы игнорируется; finished один импульс на запуск
//  12. После всех аварий сценарии снова работают; clear обнуляет rd_word, статус, список
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps
`include "i2c_line_mon.svh"

module i2c_seq_tb;

    localparam int  CLK_FREQ   = 50_000_000;
    localparam real CLK_NS     = 1.0e9 / CLK_FREQ;
    localparam int  LEN_W      = 8;
    localparam int  N_FOUND    = 8;
    localparam int  POLL_MAX   = 100;

    localparam logic [2:0] SC_SCAN = 0, SC_DT_TEMP = 1, SC_DT_TOS = 2, SC_EE_READ = 3, SC_EE_WRITE = 4, SC_ABSENT = 5;

    class Counters;
        int txn = 0, nack = 0, err = 0, ev_start = 0, finished = 0;

        function void clear();
            txn = 0;  nack = 0;  err = 0;  ev_start = 0;  finished = 0;
        endfunction
    endclass

    class Check;
        int errors = 0;

        function void err(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction

        function void equal(string name, int got, int exp);
            if (got !== exp) err($sformatf("%s: %0d (0x%h), expected %0d (0x%h)", name, got, got, exp, exp));
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic             clk = 1'b0, rst_n = 1'b0;
    logic             start = 1'b0;
    logic [2:0]       script = '0;
    logic             busy, finished, seq_error, scan_active, expect_nack;
    logic             cmd_valid, cmd_ready, cmd_read, cmd_stop;
    logic [6:0]       cmd_addr;
    logic [LEN_W-1:0] cmd_len;
    logic [7:0]       wr_data, rd_data;
    logic             wr_valid, wr_ready, rd_valid, rd_ready, rd_last;
    logic             done, res_nack_addr, res_nack_data, res_timeout, res_arb;
    logic [LEN_W-1:0] res_len;
    logic             ev_start, ev_stop, ev_ack, ev_nack;
    tri1              scl, sda;

    logic [15:0] rd_word;
    logic [6:0]  last_addr, found_addr;
    logic [7:0]  last_status;
    logic        txn_p, nack_p, err_p;
    logic [$clog2(N_FOUND)-1:0] found_idx = '0;
    logic [$clog2(N_FOUND):0]   found_cnt;
    logic        clear = 1'b0;

    always #(CLK_NS / 2) clk = ~clk;

    i2c_master #(
        .CLK_FREQ (CLK_FREQ), .SCL_FREQ (100_000), .TIMEOUT_US (1_000), .LEN_W (LEN_W)
    ) u_i2c (
        .clk (clk), .rst_n (rst_n),
        .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_addr (cmd_addr), .cmd_read (cmd_read),
        .cmd_len (cmd_len), .cmd_stop (cmd_stop),
        .wr_data (wr_data), .wr_valid (wr_valid), .wr_ready (wr_ready),
        .rd_data (rd_data), .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_last (rd_last),
        .done (done), .res_len (res_len), .res_nack_addr (res_nack_addr), .res_nack_data (res_nack_data),
        .res_timeout (res_timeout), .res_arb (res_arb),
        .ev_start (ev_start), .ev_stop (ev_stop), .ev_ack (ev_ack), .ev_nack (ev_nack),
        .i2c_scl (scl), .i2c_sda (sda)
    );

    i2c_seq #(
        .LEN_W (LEN_W), .POLL_MAX (POLL_MAX)
    ) dut (
        .clk (clk), .rst_n (rst_n),
        .start (start), .script (script),
        .busy (busy), .finished (finished), .error (seq_error), .scan_active (scan_active), .expect_nack (expect_nack),
        .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_addr (cmd_addr), .cmd_read (cmd_read),
        .cmd_len (cmd_len), .cmd_stop (cmd_stop),
        .wr_data (wr_data), .wr_valid (wr_valid), .wr_ready (wr_ready),
        .done (done), .res_nack_addr (res_nack_addr), .res_nack_data (res_nack_data),
        .res_timeout (res_timeout), .res_arb (res_arb)
    );

    i2c_res_parser #(.N_FOUND (N_FOUND)) u_parse (
        .clk (clk), .rst_n (rst_n),
        .cmd_fire (cmd_valid && cmd_ready), .cmd_addr (cmd_addr), .cmd_read (cmd_read),
        .done (done), .res_nack_addr (res_nack_addr), .res_nack_data (res_nack_data),
        .res_timeout (res_timeout), .res_arb (res_arb), .expect_nack (expect_nack),
        .rd_data (rd_data), .rd_valid (rd_valid), .rd_ready (rd_ready),
        .found_en (scan_active), .found_clear (start && script == SC_SCAN && !busy), .clear (clear),
        .rd_word (rd_word), .last_addr (last_addr), .last_status (last_status),
        .txn_p (txn_p), .nack_p (nack_p), .err_p (err_p),
        .found_idx (found_idx), .found_addr (found_addr), .found_cnt (found_cnt)
    );

    i2c_slave_model #(.ADDR (7'h48)) s48 (.scl (scl), .sda (sda));
    i2c_slave_model #(
        .ADDR (7'h50), .ADDR_MASK (7'h7C), .MEM_BITS (10), .BLOCK_BITS (2), .PAGE_SIZE (16),
        .WRITE_CYCLE_NS (5_000_000.0)
    ) s50 (.scl (scl), .sda (sda));

    Check      chk = new();
    Counters   cnt = new();
    I2CLineMon mon = new();

    always @(posedge scl) if (rst_n) mon.scl_rise($realtime, sda);
    always @(negedge scl) if (rst_n) mon.scl_fall($realtime, sda);
    always @(posedge sda) if (rst_n) mon.sda_rise($realtime, scl);
    always @(negedge sda) if (rst_n) mon.sda_fall($realtime, scl);

    always @(posedge clk) begin
        if (txn_p)    cnt.txn++;
        if (nack_p)   cnt.nack++;
        if (err_p)    cnt.err++;
        if (ev_start) cnt.ev_start++;
        if (finished) cnt.finished++;
    end

    // -------------------------------------------------------------------------
    // Запуск сценария и ожидание конца
    // -------------------------------------------------------------------------
    int starts0, stops0;

    task automatic run_script(input logic [2:0] sc, input int limit_us = 60_000);
        int guard = limit_us * 50;
        cnt.clear();
        starts0 = mon.n_start;
        stops0  = mon.n_stop;
        @(posedge clk);
        script <= sc;
        start  <= 1'b1;
        @(posedge clk);
        start  <= 1'b0;
        while (cnt.finished == 0 && guard > 0) begin
            @(posedge clk);
            guard--;
        end
        if (guard == 0) chk.err($sformatf("script %0d did not finish in %0d us", sc, limit_us));
        repeat (20) @(posedge clk);
    endtask

    task automatic expect_idle_bus(input string name);
        if (scl !== 1'b1 || sda !== 1'b1) chk.err($sformatf("%s: bus is not idle SCL=%b SDA=%b", name, scl, sda));
        if (busy) chk.err($sformatf("%s: sequencer still busy", name));
    endtask

    // список найденных целиком
    task automatic get_found(output logic [6:0] list [$]);
        list.delete();
        for (int i = 0; i < found_cnt; i++) begin
            found_idx = i[$clog2(N_FOUND)-1:0];
            @(posedge clk);
            #1;
            list.push_back(found_addr);
        end
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        logic [6:0] list [$];
        logic [7:0] pattern1, pattern2;
        logic [15:0] word_before;
        $timeformat(-6, 2, " us", 0);
        for (int i = 0; i < 256; i++)  s48.mem[i] = 8'hEE;
        for (int i = 0; i < 1024; i++) s50.mem[i] = 8'hEE;
        s48.mem[0] = 8'h19;  s48.mem[1] = 8'h60;                // температура 25.375 C
        s48.mem[3] = 8'h50;  s48.mem[4] = 8'h00;                // Tos 80 C
        s50.mem[0] = 8'h11;  s50.mem[1] = 8'h22;  s50.mem[2] = 8'h33;  s50.mem[3] = 8'h44;
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        $display("[%0t] 1. SCAN", $time);
        run_script(SC_SCAN);
        chk.equal("scan: transactions", cnt.txn, 112);
        chk.equal("scan: NACK", cnt.nack, 107);
        chk.equal("scan: errors", cnt.err, 0);
        chk.equal("scan: sequencer error", seq_error, 0);
        chk.equal("scan: found_cnt", found_cnt, 5);
        chk.equal("scan: last address", last_addr, 7'h77);
        get_found(list);
        if (list != '{7'h48, 7'h50, 7'h51, 7'h52, 7'h53}) chk.err($sformatf("scan: found list %p", list));
        expect_idle_bus("scan");
        run_script(SC_SCAN);
        chk.equal("second scan: found_cnt", found_cnt, 5);
        chk.equal("second scan: transactions", cnt.txn, 112);

        $display("[%0t] 2. DT_TEMP", $time);
        run_script(SC_DT_TEMP);
        chk.equal("DT_TEMP: rd_word", rd_word, 16'h1960);
        chk.equal("DT_TEMP: transactions", cnt.txn, 2);
        chk.equal("DT_TEMP: errors", cnt.err, 0);
        chk.equal("DT_TEMP: last address", last_addr, 7'h48);
        chk.equal("DT_TEMP: status", last_status, 8'h80);
        chk.equal("DT_TEMP: START on the wires", mon.n_start - starts0, 2);
        chk.equal("DT_TEMP: STOP on the wires", mon.n_stop - stops0, 1);
        chk.equal("DT_TEMP: START events", cnt.ev_start, 2);
        expect_idle_bus("DT_TEMP");

        $display("[%0t] 3. DT_TOS", $time);
        run_script(SC_DT_TOS);
        chk.equal("DT_TOS: rd_word", rd_word, 16'h5000);
        chk.equal("DT_TOS: errors", cnt.err, 0);

        $display("[%0t] 4. EE_READ", $time);
        run_script(SC_EE_READ);
        chk.equal("EE_READ: rd_word", rd_word, 16'h3344);
        chk.equal("EE_READ: transactions", cnt.txn, 2);
        chk.equal("EE_READ: last address", last_addr, 7'h50);
        chk.equal("EE_READ: errors", cnt.err, 0);

        $display("[%0t] 5. EE_WRITE twice (ACK polling during the 5 ms write cycle)", $time);
        pattern1 = 8'h5A + 8'(dut.run_cnt);
        run_script(SC_EE_WRITE);
        chk.equal("EE_WRITE: cell", s50.mem[8'hF0], pattern1);
        chk.equal("EE_WRITE: read back", rd_word[7:0], pattern1);
        chk.equal("EE_WRITE: errors", cnt.err, 0);
        chk.equal("EE_WRITE: sequencer error", seq_error, 0);
        if (cnt.nack == 0) chk.err("EE_WRITE: no NACK while polling");
        chk.equal("EE_WRITE: transactions = write + polls + ACK + pointer + read", cnt.txn, 1 + cnt.nack + 1 + 1 + 1);
        $display("[%0t]    polls with NACK: %0d", $time, cnt.nack);
        pattern2 = 8'h5A + 8'(dut.run_cnt);
        if (pattern2 == pattern1) chk.err("EE_WRITE: pattern did not change");
        run_script(SC_EE_WRITE);
        chk.equal("EE_WRITE #2: cell", s50.mem[8'hF0], pattern2);
        chk.equal("EE_WRITE #2: read back", rd_word[7:0], pattern2);
        chk.equal("EE_WRITE #2: errors", cnt.err, 0);

        $display("[%0t] 6. ABSENT", $time);
        run_script(SC_ABSENT);
        chk.equal("ABSENT: NACK", cnt.nack, 1);
        chk.equal("ABSENT: errors", cnt.err, 0);
        chk.equal("ABSENT: sequencer error", seq_error, 0);
        chk.equal("ABSENT: status", last_status, 8'h01);
        chk.equal("ABSENT: last address", last_addr, 7'h2A);
        expect_idle_bus("ABSENT");

        $display("[%0t] 7. Sensor is missing", $time);
        word_before = rd_word;
        s48.absent = 1'b1;
        run_script(SC_DT_TEMP);
        chk.equal("no sensor: sequencer error", seq_error, 1);
        chk.equal("no sensor: transactions", cnt.txn, 1);
        chk.equal("no sensor: errors", cnt.err, 1);
        chk.equal("no sensor: rd_word is the old one", rd_word, word_before);
        expect_idle_bus("no sensor");
        s48.absent = 1'b0;
        run_script(SC_DT_TEMP);
        chk.equal("sensor is back: sequencer error", seq_error, 0);
        chk.equal("sensor is back: rd_word", rd_word, 16'h1960);

        $display("[%0t] 8. NACK on a data byte", $time);
        s50.wr_limit = 1;
        run_script(SC_EE_WRITE);
        chk.equal("NACK data: sequencer error", seq_error, 1);
        chk.equal("NACK data: errors", cnt.err, 1);
        chk.equal("NACK data: status", last_status, 8'h02);
        expect_idle_bus("NACK data");
        s50.wr_limit = 1_000_000;

        $display("[%0t] 9. Timeout: sensor holds SCL", $time);
        mon.enable = 1'b0;
        s48.stretch_ns = 3_000_000;
        run_script(SC_DT_TEMP);
        chk.equal("timeout: sequencer error", seq_error, 1);
        chk.equal("timeout: errors", cnt.err, 1);
        chk.equal("timeout: status", last_status, 8'h04);
        s48.stretch_ns = 0;
        #(4_000_000.0);
        expect_idle_bus("timeout");
        mon.enable = 1'b1;

        $display("[%0t] 10. EEPROM disappears while polling", $time);
        fork
            begin
                run_script(SC_EE_WRITE, 80_000);
            end
            begin
                wait (dut.step_n == 2'd1);
                s50.absent = 1'b1;
            end
        join
        s50.absent = 1'b0;
        chk.equal("poll limit: sequencer error", seq_error, 1);
        chk.equal("poll limit: errors", cnt.err, 1);
        chk.equal("poll limit: NACK count", cnt.nack, POLL_MAX + 1);
        expect_idle_bus("poll limit");
        #(6_000_000.0);                                     // цикл записи закончился

        $display("[%0t] 11. start while busy is ignored", $time);
        cnt.clear();
        @(posedge clk);
        script <= SC_DT_TEMP;
        start  <= 1'b1;
        @(posedge clk);
        start  <= 1'b0;
        repeat (500) @(posedge clk);
        script <= SC_ABSENT;
        start  <= 1'b1;                                     // во время работы
        @(posedge clk);
        start  <= 1'b0;
        while (cnt.finished == 0) @(posedge clk);
        repeat (20) @(posedge clk);
        repeat (2000) @(posedge clk);
        chk.equal("busy start: finished pulses", cnt.finished, 1);
        chk.equal("busy start: transactions", cnt.txn, 2);
        chk.equal("busy start: rd_word", rd_word, 16'h1960);

        $display("[%0t] 12. Everything works after the failures; clear", $time);
        run_script(SC_DT_TOS);
        chk.equal("after failures: rd_word", rd_word, 16'h5000);
        chk.equal("after failures: errors", cnt.err, 0);
        run_script(SC_EE_READ);
        chk.equal("after failures: EE_READ", rd_word, {s50.mem[2], s50.mem[3]});
        @(posedge clk);
        clear <= 1'b1;
        @(posedge clk);
        clear <= 1'b0;
        @(posedge clk);
        chk.equal("clear: rd_word", rd_word, 0);
        chk.equal("clear: last_status", last_status, 0);
        chk.equal("clear: found_cnt", found_cnt, 0);

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d (testbench) + %0d (bus monitor)", chk.errors, mon.errors);
        if (chk.errors == 0 && mon.errors == 0) $display("TEST PASSED");
        else                                    $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #400ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
