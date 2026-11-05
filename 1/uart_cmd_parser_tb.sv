// =============================================================================
// Тестбенч для uart_cmd_parser (команды МУ: Старт 0x01, Стоп 0x02)
//
// Классы:
//   CmdModel - эталонная модель: по входному байту предсказывает импульсы
//              cmd_start / cmd_stop / garbage и уровень running.
// Проверки:
//   1. После сброса: running = 0, импульсов нет
//   2. Последовательность Старт, мусор, Старт (повтор), Стоп, Стоп (повтор), Старт
//   3. Все 256 значений байта по очереди
//   4. 2000 случайных байт (половина - команды), паузы 0..3 такта
//   Каждый такт выходы DUT сравниваются с моделью: импульс на следующем такте
//   после байта, длительность 1 такт, running меняется вместе с импульсом.
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module uart_cmd_parser_tb;

    localparam real CLK_NS = 20.0;

    class CmdModel;
        bit start, stop, garbage;       // ожидаемые импульсы в следующем такте
        bit running = 0;
        int n_start = 0, n_stop = 0, n_garbage = 0;

        // valid = 0: в следующем такте импульсов нет
        function void step(bit valid, logic [7:0] data);
            start   = valid && data == 8'h01;
            stop    = valid && data == 8'h02;
            garbage = valid && !start && !stop;
            if (start) begin running = 1; n_start++; end
            if (stop)  begin running = 0; n_stop++;  end
            if (garbage) n_garbage++;
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk   = 1'b0;
    logic       rst_n = 1'b0;
    logic [7:0] data  = '0;
    logic       valid = 1'b0;
    logic       cmd_start, cmd_stop, garbage, running;

    always #(CLK_NS / 2) clk = ~clk;

    uart_cmd_parser dut (.*);

    CmdModel model  = new();
    int      errors = 0;
    int      cycles = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    // сравнение с моделью: выходы этого такта - результат входа прошлого такта
    always @(posedge clk) begin
        if (rst_n) begin
            cycles++;
            if ({cmd_start, cmd_stop, garbage, running} !==
                {model.start, model.stop, model.garbage, model.running})
                fail($sformatf("start/stop/garbage/running = %b%b%b%b, expected %b%b%b%b",
                               cmd_start, cmd_stop, garbage, running,
                               model.start, model.stop, model.garbage, model.running));
            model.step(valid, data);
        end
    end

    task automatic send(input logic [7:0] b, input int max_gap = 0);
        data  <= b;
        valid <= 1'b1;
        @(posedge clk);
        valid <= 1'b0;
        data  <= 'x;
        repeat ($urandom_range(0, max_gap)) @(posedge clk);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    localparam logic [7:0] SEQ2[6] = '{8'h01, 8'h55, 8'h01, 8'h02, 8'h02, 8'h01};

    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        $display("[%0t] 1. Reset state", $time);
        repeat (3) @(posedge clk);

        $display("[%0t] 2. Start, garbage, Start, Stop, Stop, Start", $time);
        foreach (SEQ2[i]) send(SEQ2[i], 2);
        repeat (3) @(posedge clk);

        $display("[%0t] 3. All 256 byte values", $time);
        for (int b = 0; b < 256; b++) send(8'(b));

        $display("[%0t] 4. 2000 random bytes", $time);
        for (int i = 0; i < 2000; i++)
            send(($urandom_range(0, 1)) ? 8'($urandom_range(1, 2)) : 8'($urandom), 3);
        repeat (3) @(posedge clk);

        $display("------------------------------------------------------------");
        $display("Start: %0d, Stop: %0d, garbage: %0d, cycles: %0d, ERRORS: %0d",
                 model.n_start, model.n_stop, model.n_garbage, cycles, errors);
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
