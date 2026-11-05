// =============================================================================
// Тестбенч для bin2bcd: перевод числа в десятичные цифры
//
// Проверки:
//   1. Все 1024 значения 10-битного входа: результат равен десятичной записи числа
//      (цифры считаются делением в тестбенче), перевод занимает ровно IN_W + 1 такт
//      от start до конца busy
//   2. bcd не меняется, пока перевод идёт, и обновляется одним тактом после его окончания
//   3. start во время busy игнорируется (число не подменяется)
//   4. Постоянный перевод со start = !busy: значения, меняющиеся случайным образом, всегда
//      приводят к верному результату после окончания очередного перевода
//   5. Узкий экземпляр (IN_W = 8, DIGITS = 3): все 256 значений
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module bin2bcd_tb;

    localparam real CLK_NS = 20.0;

    class Scoreboard;
        int errors = 0;

        function void err(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    logic        clk = 1'b0, rst_n = 1'b0;
    logic        start = 1'b0;
    logic [9:0]  bin = '0;
    logic        busy;
    logic [15:0] bcd;
    logic        start8 = 1'b0;
    logic [7:0]  bin8 = '0;
    logic        busy8;
    logic [11:0] bcd8;

    always #(CLK_NS / 2) clk = ~clk;

    bin2bcd #(.IN_W (10), .DIGITS (4)) dut (
        .clk (clk), .rst_n (rst_n), .start (start), .bin (bin), .busy (busy), .bcd (bcd)
    );

    bin2bcd #(.IN_W (8), .DIGITS (3)) dut8 (
        .clk (clk), .rst_n (rst_n), .start (start8), .bin (bin8), .busy (busy8), .bcd (bcd8)
    );

    Scoreboard sb = new();

    // десятичная запись числа в виде BCD
    function automatic logic [15:0] to_bcd(int v);
        logic [15:0] r;
        for (int i = 0; i < 4; i++) begin
            r[4*i +: 4] = 4'(v % 10);
            v = v / 10;
        end
        return r;
    endfunction

    // один перевод; возвращает число тактов от start до снятия busy
    task automatic convert(input int v, output int cycles, output logic [15:0] result);
        @(negedge clk);
        bin   = 10'(v);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
        cycles = 1;
        while (busy) begin
            @(negedge clk);
            cycles++;
        end
        result = bcd;
    endtask

    // bcd меняется не раньше окончания и сразу целиком
    logic [15:0] bcd_q = '0;
    logic        busy_q = 1'b0;
    always @(posedge clk) begin
        if (rst_n && busy_q && busy && bcd !== bcd_q)
            sb.err($sformatf("bcd changed while busy: %h -> %h", bcd_q, bcd));
        bcd_q  <= bcd;
        busy_q <= busy;
    end

    initial begin
        int          cycles;
        logic [15:0] res;
        $timeformat(-6, 2, " us", 0);
        #(CLK_NS * 3);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        // ---------------------------------------------------------------- 1
        for (int v = 0; v < 1024; v++) begin
            convert(v, cycles, res);
            if (res !== to_bcd(v)) sb.err($sformatf("%0d -> %h, expected %h", v, res, to_bcd(v)));
            if (cycles != 11) sb.err($sformatf("%0d: conversion takes %0d clocks, expected 11", v, cycles));
        end

        // ---------------------------------------------------------------- 3
        @(negedge clk);
        bin = 10'd789;  start = 1'b1;
        @(negedge clk);
        bin = 10'd123;                                          // start ещё 1, busy = 1: новое число не принимается
        repeat (3) @(negedge clk);
        start = 1'b0;
        while (busy) @(negedge clk);
        if (bcd !== to_bcd(789)) sb.err($sformatf("start during busy changed the result: %h", bcd));

        // ---------------------------------------------------------------- 4
        for (int k = 0; k < 200; k++) begin
            int v;
            v = $urandom_range(0, 1023);
            bin = 10'(v);
            @(negedge clk);
            start = !busy;
            if (start) begin
                @(negedge clk);
                start = 1'b0;
                while (busy) @(negedge clk);
                if (bcd !== to_bcd(v)) sb.err($sformatf("continuous: %0d -> %h", v, bcd));
            end
        end

        // ---------------------------------------------------------------- 5
        for (int v = 0; v < 256; v++) begin
            @(negedge clk);
            bin8 = 8'(v);
            start8 = 1'b1;
            @(negedge clk);
            start8 = 1'b0;
            while (busy8) @(negedge clk);
            if (bcd8 !== to_bcd(v)) sb.err($sformatf("8-bit: %0d -> %h", v, bcd8));
        end

        repeat (5) @(posedge clk);
        if (sb.errors == 0) $display("TEST PASSED");
        else                $display("TEST FAILED: %0d errors", sb.errors);
        $finish;
    end

endmodule
