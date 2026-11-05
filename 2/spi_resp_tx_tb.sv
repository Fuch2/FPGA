// =============================================================================
// Тестбенч для spi_resp_tx (ответ МИ: STATUS, D_H, D_L, CHK)
//
// Тестбенч играет роль spi_slave: подаёт frame_start и случайно готовит tx_ready,
// забирает байты по handshake valid/ready.
// Классы:
//   RespModel - эталон ответа: четыре байта слова и контрольная сумма.
// Проверки:
//   1. После сброса: tx_valid = 0
//   2. Кадр: ровно 4 байта STATUS, D_H, D_L, CHK = ~(STATUS ^ D_H ^ D_L)
//   3. Изменение status и data после frame_start на ответ не влияет
//      (слово защёлкнуто в момент спада CS_N)
//   4. tx_ready случайно (вплоть до 30 тактов ожидания): байты не теряются и не повторяются
//   5. После 4 байт tx_valid = 0 до следующего frame_start
//   6. frame_start посреди ответа: ответ начинается заново с новыми данными
//   7. 2000 случайных кадров
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module spi_resp_tx_tb;

    localparam real CLK_NS = 20.0;

    typedef logic [7:0] bytes_t[$];

    class RespModel;
        function bytes_t word(logic [7:0] s, logic [15:0] d);
            bytes_t w;
            w.delete();
            w.push_back(s);
            w.push_back(d[15:8]);
            w.push_back(d[7:0]);
            w.push_back(~(s ^ d[15:8] ^ d[7:0]));
            return w;
        endfunction
    endclass

    logic        clk   = 1'b0;
    logic        rst_n = 1'b0;
    logic        frame_start = 1'b0;
    logic [7:0]  status = '0;
    logic [15:0] data   = '0;
    logic [7:0]  tx_data;
    logic        tx_valid;
    logic        tx_ready = 1'b0;

    always #(CLK_NS / 2) clk = ~clk;

    spi_resp_tx dut (.*);

    RespModel model = new();
    int       errors = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    // принятые байты (handshake valid & ready на фронте clk)
    bytes_t got;
    always @(posedge clk) if (rst_n && tx_valid && tx_ready) got.push_back(tx_data);

    function automatic bit same(bytes_t a, bytes_t b);
        if (a.size() != b.size()) return 0;
        foreach (a[i]) if (a[i] !== b[i]) return 0;
        return 1;
    endfunction

    // кадр: status/data действительны в момент frame_start, потом меняются; max_wait - случайные паузы ready
    task automatic frame(input string name, input logic [7:0] s, input logic [15:0] d,
                         input int max_wait = 0, input bit change_after = 1);
        bytes_t exp_w = model.word(s, d);
        int     guard = 0;
        got.delete();
        status      <= s;
        data        <= d;
        @(posedge clk);
        frame_start <= 1'b1;
        @(posedge clk);
        frame_start <= 1'b0;
        if (change_after) begin
            status <= ~s;
            data   <= ~d;
        end
        while (got.size() < 4 && guard < 4 * (max_wait + 5) + 50) begin
            tx_ready <= ($urandom_range(0, max_wait) == 0);
            @(posedge clk);
            guard++;
        end
        tx_ready <= 1'b1;
        repeat (6) @(posedge clk);      // лишних байтов быть не должно
        tx_ready <= 1'b0;
        if (!same(got, exp_w))
            fail($sformatf("%s: bytes %p, expected %p", name, got, exp_w));
        if (tx_valid !== 1'b0)
            fail($sformatf("%s: tx_valid = 1 after the 4th byte", name));
    endtask

    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        $display("[%0t] 1. Reset state", $time);
        if (tx_valid !== 1'b0) fail("tx_valid after reset");

        $display("[%0t] 2. Frame", $time);
        frame("basic", 8'h83, 16'h1234, 0, 0);
        frame("zeros", 8'h80, 16'h0000, 0, 0);
        frame("ones",  8'hFF, 16'hFFFF, 0, 0);

        $display("[%0t] 3. status and data change after frame_start", $time);
        frame("latched", 8'h81, 16'hA55A, 0, 1);

        $display("[%0t] 4. Random tx_ready", $time);
        repeat (30) frame("slow", 8'($urandom), 16'($urandom), 30, 1);

        $display("[%0t] 6. frame_start in the middle of a response", $time);
        begin
            automatic bytes_t exp_w = model.word(8'h82, 16'hBEEF);
            got.delete();
            status <= 8'h81; data <= 16'h0102;
            @(posedge clk);
            frame_start <= 1'b1;
            @(posedge clk);
            frame_start <= 1'b0;
            tx_ready <= 1'b1;
            @(posedge clk);                 // один байт ушёл
            tx_ready <= 1'b0;
            status <= 8'h82; data <= 16'hBEEF;
            @(posedge clk);
            frame_start <= 1'b1;
            @(posedge clk);
            frame_start <= 1'b0;
            got.delete();
            tx_ready <= 1'b1;
            repeat (8) @(posedge clk);
            tx_ready <= 1'b0;
            if (!same(got, exp_w)) fail($sformatf("restart: bytes %p, expected %p", got, exp_w));
        end

        $display("[%0t] 7. 2000 random frames", $time);
        for (int k = 0; k < 2000; k++)
            frame($sformatf("rand%0d", k), 8'($urandom), 16'($urandom), $urandom_range(0, 5), $urandom_range(0, 1));

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d", errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #20ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
