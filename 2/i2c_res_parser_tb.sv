// =============================================================================
// Тестбенч для i2c_res_parser (разбор итога транзакций и принятых данных)
//
// Входы (команда, поток чтения, итог, управление) задаются случайно и направленно;
// эталон считает то же самое по тактам и сравнивается с выходами в каждом такте.
// Классы:
//   ParserModel - эталон: rd_word, last_addr, last_status, список найденных,
//                 импульсы txn_p / nack_p / err_p (на такт позже done).
// Проверки:
//   1. После сброса: всё нули, список пуст, rd_ready = 1
//   2. Направленно: чтение двух байт -> rd_word = {первый, второй}; итог с NACK адреса,
//      NACK данных, timeout, arb - статус и импульсы; ожидаемый NACK не ошибка
//   3. Список найденных: пополняется при found_en и успешном итоге; не пополняется
//      при NACK, timeout, arb; после 8 записей не растёт; found_clear и clear очищают
//   4. 30000 тактов случайных событий - выходы совпадают с эталоном в каждом такте
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module i2c_res_parser_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  N_FOUND  = 8;

    class ParserModel;
        logic [6:0]  cur_addr = '0;
        bit          cur_read = 0;
        logic [15:0] rd_word = '0;
        logic [6:0]  last_addr = '0;
        logic [7:0]  last_status = '0;
        logic [6:0]  found [N_FOUND];
        int          found_cnt = 0;
        bit          txn_p = 0, nack_p = 0, err_p = 0;

        function new();
            foreach (found[i]) found[i] = '0;
        endfunction

        function void step(bit cmd_fire, logic [6:0] cmd_addr, bit cmd_read,
                           bit done, bit nack_addr, bit nack_data, bit timeout, bit arb, bit expect_nack,
                           bit rd_valid, logic [7:0] rd_data, bit found_en, bit found_clear, bit clear);
            bit any_nack = nack_addr || nack_data;
            bit any_err  = nack_data || timeout || arb || (nack_addr && !expect_nack);
            logic [6:0] old_addr = cur_addr;      // в DUT cmd_fire и done в одном такте видят прежний адрес
            bit         old_read = cur_read;
            txn_p = 0;  nack_p = 0;  err_p = 0;
            if (cmd_fire) begin cur_addr = cmd_addr;  cur_read = cmd_read; end
            if (rd_valid) rd_word = {rd_word[7:0], rd_data};
            if (done) begin
                last_addr   = old_addr;
                last_status = {old_read, 3'b000, arb, timeout, nack_data, nack_addr};
                txn_p  = 1;
                nack_p = nack_addr;
                err_p  = any_err;
                if (found_en && !any_nack && !timeout && !arb && found_cnt != N_FOUND) begin
                    found[found_cnt] = old_addr;
                    found_cnt++;
                end
            end
            if (clear) begin
                rd_word = '0;  last_addr = '0;  last_status = '0;
            end
            if (clear || found_clear) begin
                found_cnt = 0;
                foreach (found[i]) found[i] = '0;
            end
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic        clk = 1'b0, rst_n = 1'b0;
    logic        cmd_fire = 0, cmd_read = 0, done = 0;
    logic [6:0]  cmd_addr = '0;
    logic        res_nack_addr = 0, res_nack_data = 0, res_timeout = 0, res_arb = 0, expect_nack = 0;
    logic [7:0]  rd_data = '0;
    logic        rd_valid = 0, rd_ready;
    logic        found_en = 0, found_clear = 0, clear = 0;
    logic [15:0] rd_word;
    logic [6:0]  last_addr, found_addr;
    logic [7:0]  last_status;
    logic        txn_p, nack_p, err_p;
    logic [$clog2(N_FOUND)-1:0] found_idx = '0;
    logic [$clog2(N_FOUND):0]   found_cnt;

    always #(CLK_NS / 2) clk = ~clk;

    i2c_res_parser #(.N_FOUND (N_FOUND)) dut (.*);

    ParserModel model  = new();
    int         errors = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    always @(posedge clk) begin
        if (rst_n) begin
            if (rd_word !== model.rd_word || last_addr !== model.last_addr || last_status !== model.last_status)
                fail($sformatf("rd_word %h last_addr %h last_status %h, expected %h %h %h",
                               rd_word, last_addr, last_status, model.rd_word, model.last_addr, model.last_status));
            if (found_cnt !== model.found_cnt)
                fail($sformatf("found_cnt %0d, expected %0d", found_cnt, model.found_cnt));
            if (found_addr !== model.found[found_idx])
                fail($sformatf("found_addr[%0d] = %h, expected %h", found_idx, found_addr, model.found[found_idx]));
            if (txn_p !== model.txn_p || nack_p !== model.nack_p || err_p !== model.err_p)
                fail($sformatf("pulses txn %b nack %b err %b, expected %b %b %b",
                               txn_p, nack_p, err_p, model.txn_p, model.nack_p, model.err_p));
            if (rd_ready !== 1'b1) fail("rd_ready != 1");
            model.step(cmd_fire, cmd_addr, cmd_read, done, res_nack_addr, res_nack_data, res_timeout, res_arb,
                       expect_nack, rd_valid, rd_data, found_en, found_clear, clear);
        end
    end

    // одна транзакция: команда, чтение n байт, итог
    task automatic txn(input logic [6:0] a, input bit rd, input int n, input bit nack_a, input bit nack_d,
                       input bit to, input bit arb, input bit exp_nack, input bit scan);
        @(posedge clk);
        cmd_addr <= a;  cmd_read <= rd;  cmd_fire <= 1'b1;  found_en <= scan;  expect_nack <= exp_nack;
        @(posedge clk);
        cmd_fire <= 1'b0;
        repeat (n) begin
            rd_data  <= $urandom;
            rd_valid <= 1'b1;
            @(posedge clk);
            rd_valid <= 1'b0;
            @(posedge clk);
        end
        res_nack_addr <= nack_a;  res_nack_data <= nack_d;  res_timeout <= to;  res_arb <= arb;
        done <= 1'b1;
        @(posedge clk);
        done <= 1'b0;
        {res_nack_addr, res_nack_data, res_timeout, res_arb} <= '0;
        repeat (2) @(posedge clk);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        $display("[%0t] 1. Reset state", $time);
        if (rd_word !== 0 || last_addr !== 0 || last_status !== 0 || found_cnt !== 0 || rd_ready !== 1'b1)
            fail("outputs after reset are not 0");

        $display("[%0t] 2. Directed results", $time);
        txn(7'h48, 1, 2, 0, 0, 0, 0, 0, 0);                 // чтение двух байт
        if (last_status !== 8'h80) fail($sformatf("read status %h, expected 80", last_status));
        txn(7'h2A, 0, 0, 1, 0, 0, 0, 0, 0);                 // NACK адреса, не ожидали -> ошибка
        if (last_status !== 8'h01 || last_addr !== 7'h2A) fail("NACK address status");
        txn(7'h2A, 0, 0, 1, 0, 0, 0, 1, 0);                 // ожидаемый NACK
        txn(7'h50, 0, 0, 0, 1, 0, 0, 0, 0);                 // NACK данных
        if (last_status !== 8'h02) fail("NACK data status");
        txn(7'h50, 1, 0, 0, 0, 1, 0, 0, 0);                 // timeout
        if (last_status !== 8'h84) fail("timeout status");
        txn(7'h50, 0, 0, 0, 0, 0, 1, 0, 0);                 // arb
        if (last_status !== 8'h08) fail("arb status");

        $display("[%0t] 3. Found list", $time);
        @(posedge clk);
        clear <= 1'b1;
        @(posedge clk);
        clear <= 1'b0;
        repeat (2) @(posedge clk);
        for (int k = 0; k < 11; k++)                        // 11 адресов, из них часть не годится
            txn(7'h10 + 7'(k), 0, 0, (k == 3), (k == 5), (k == 7), (k == 8), (k == 3), 1);
        if (found_cnt !== 7) fail($sformatf("found_cnt %0d, expected 7 (4 of 11 are unsuitable)", found_cnt));
        for (int k = 0; k < 6; k++) txn(7'h30 + 7'(k), 0, 0, 0, 0, 0, 0, 0, 1);
        if (found_cnt !== N_FOUND) fail($sformatf("found_cnt %0d after overflow, expected %0d", found_cnt, N_FOUND));
        found_idx <= 3'd7;
        repeat (2) @(posedge clk);
        if (found_addr !== model.found[7]) fail("found_addr[7]");
        @(posedge clk);
        found_clear <= 1'b1;
        @(posedge clk);
        found_clear <= 1'b0;
        repeat (2) @(posedge clk);
        if (found_cnt !== 0) fail("found_clear did not empty the list");

        $display("[%0t] 4. 30000 clocks of random events", $time);
        repeat (30000) begin
            cmd_fire      <= ($urandom_range(0, 7) == 0);
            cmd_addr      <= $urandom;
            cmd_read      <= $urandom;
            rd_valid      <= ($urandom_range(0, 5) == 0);
            rd_data       <= $urandom;
            done          <= ($urandom_range(0, 9) == 0);
            res_nack_addr <= ($urandom_range(0, 5) == 0);
            res_nack_data <= ($urandom_range(0, 9) == 0);
            res_timeout   <= ($urandom_range(0, 14) == 0);
            res_arb       <= ($urandom_range(0, 14) == 0);
            expect_nack   <= $urandom;
            found_en      <= ($urandom_range(0, 3) != 0);
            found_clear   <= ($urandom_range(0, 399) == 0);
            clear         <= ($urandom_range(0, 799) == 0);
            found_idx     <= $urandom;
            @(posedge clk);
        end
        {cmd_fire, rd_valid, done, found_clear, clear} <= '0;
        repeat (3) @(posedge clk);

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d", errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #5ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
