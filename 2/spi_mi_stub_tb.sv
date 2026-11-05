// =============================================================================
// Тестбенч для spi_mi_stub (заглушка МИ: Старт, Измерение, Стоп, счётчик вместо ДТ)
//
// MEAS_US уменьшен до 2 мкс (100 тактов), DATA_INIT = 1, DATA_STEP = 1.
// Классы:
//   StubModel - эталон по тактам: RUN / VALID / EMPTY / BUSY, значение, счётчик,
//               meas_done; поведение «Измерения» во время BUSY.
// Проверки (сравнение с моделью каждый такт):
//   1. После сброса: status = 0x80 (MARK), data = 0
//   2. Старт: RUN = 1, VALID = 0
//   3. Измерение: BUSY на 100 тактов, затем VALID = 1, data = 1, 2, 3, ...
//   4. Измерение во время BUSY игнорируется
//   5. Стоп: RUN = 0; Измерение при остановленной МИ: EMPTY = 1, VALID = 0
//   6. Повторный Старт: счётчик снова с 1, data = 0, флаги сброшены
//   7. Старт во время BUSY отменяет измерение
//   8. 20000 тактов случайных команд
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module spi_mi_stub_tb;

    localparam int  CLK_FREQ  = 50_000_000;
    localparam real CLK_NS    = 1.0e9 / CLK_FREQ;
    localparam int  MEAS_US   = 2;
    localparam int  MEAS_CLKS = (CLK_FREQ / 1_000_000) * MEAS_US;

    class StubModel;
        bit          run = 0, valid = 0, empty = 0, busy = 0;
        int          cnt = 0;
        logic [15:0] data = 0;
        logic [15:0] next_value = 16'h0001;
        bit          meas_done = 0;
        int          n_meas = 0;

        function logic [7:0] status();
            return {1'b1, 3'b000, busy, empty, valid, run};
        endfunction

        // один такт: входы, защёлкнутые DUT по фронту
        function void step(bit c_start, bit c_measure, bit c_stop);
            meas_done = 0;
            if (c_start) begin
                busy = 0; run = 1; valid = 0; empty = 0; data = 0; next_value = 16'h0001;
            end else if (c_stop) begin
                run = 0;
            end else if (!busy) begin
                cnt = 0;
                if (c_measure) begin
                    valid = 0; empty = 0; busy = 1;
                end
            end else begin
                if (cnt == MEAS_CLKS - 1) begin
                    if (run) begin
                        data = next_value;
                        next_value++;
                        valid = 1;
                    end else begin
                        empty = 1;
                    end
                    meas_done = 1;
                    n_meas++;
                    busy = 0;
                end else begin
                    cnt++;
                end
            end
        endfunction
    endclass

    logic        clk   = 1'b0;
    logic        rst_n = 1'b0;
    logic        cmd_start = 0, cmd_measure = 0, cmd_stop = 0;
    logic [7:0]  status;
    logic [15:0] data;
    logic        running, meas_done;

    always #(CLK_NS / 2) clk = ~clk;

    spi_mi_stub #(
        .CLK_FREQ  (CLK_FREQ),
        .MEAS_US   (MEAS_US),
        .DATA_INIT (16'h0001),
        .DATA_STEP (16'h0001)
    ) dut (.*);

    StubModel model = new();
    int       errors = 0;
    int       cycles = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    always @(posedge clk) begin
        if (rst_n) begin
            cycles++;
            if (status !== model.status() || data !== model.data ||
                running !== model.run || meas_done !== model.meas_done)
                fail($sformatf("status/data/run/done = %h %h %b %b, expected %h %h %b %b",
                               status, data, running, meas_done,
                               model.status(), model.data, model.run, model.meas_done));
            model.step(cmd_start, cmd_measure, cmd_stop);
        end
    end

    task automatic pulse(input bit s, input bit m, input bit p, input int wait_clks = 3);
        cmd_start   <= s;
        cmd_measure <= m;
        cmd_stop    <= p;
        @(posedge clk);
        {cmd_start, cmd_measure, cmd_stop} <= '0;
        repeat (wait_clks) @(posedge clk);
    endtask

    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        $display("[%0t] 1. Reset state", $time);
        if (status !== 8'h80 || data !== 16'h0000) fail($sformatf("after reset %h %h", status, data));

        $display("[%0t] 2. Start", $time);
        pulse(1, 0, 0);

        $display("[%0t] 3. Measure x3", $time);
        repeat (3) pulse(0, 1, 0, MEAS_CLKS + 5);
        if (data !== 16'h0003 || status !== 8'h83) fail($sformatf("after 3 measures %h %h", status, data));

        $display("[%0t] 4. Measure while BUSY", $time);
        pulse(0, 1, 0, 10);
        pulse(0, 1, 0, 10);
        repeat (MEAS_CLKS + 5) @(posedge clk);
        if (data !== 16'h0004) fail($sformatf("measure during BUSY was not ignored, data = %h", data));

        $display("[%0t] 5. Stop, measure while stopped", $time);
        pulse(0, 0, 1);
        pulse(0, 1, 0, MEAS_CLKS + 5);
        if (status !== 8'h84) fail($sformatf("EMPTY expected, status = %h", status));

        $display("[%0t] 6. Start again", $time);
        pulse(1, 0, 0);
        if (status !== 8'h81 || data !== 16'h0000) fail($sformatf("after restart %h %h", status, data));
        pulse(0, 1, 0, MEAS_CLKS + 5);
        if (data !== 16'h0001) fail($sformatf("counter did not restart, data = %h", data));

        $display("[%0t] 7. Start while BUSY", $time);
        pulse(0, 1, 0, 20);
        pulse(1, 0, 0, MEAS_CLKS + 5);
        if (status !== 8'h81) fail($sformatf("Start did not cancel the measurement, status = %h", status));

        $display("[%0t] 8. Random commands", $time);
        repeat (20000) begin
            cmd_start   <= ($urandom_range(0, 399) == 0);
            cmd_measure <= ($urandom_range(0, 59) == 0);
            cmd_stop    <= ($urandom_range(0, 399) == 0);
            @(posedge clk);
        end
        {cmd_start, cmd_measure, cmd_stop} <= '0;
        repeat (MEAS_CLKS + 5) @(posedge clk);

        $display("------------------------------------------------------------");
        $display("Measurements done: %0d, cycles: %0d, ERRORS: %0d", model.n_meas, cycles, errors);
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
