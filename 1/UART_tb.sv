// =============================================================================
// Тестбенч для UART (приёмник + передатчик)
//
// Проверки:
//   1. Состояние после сброса
//   2. Передатчик: данные и длительность битов на линии TX, поток байт без пауз
//   3. Приёмник: приём на номинальной скорости, кадры без пауз
//   4. Приёмник: отклонение скорости передающей стороны ±3 %
//   5. Приёмник: короткая помеха на линии не даёт ложного приёма
//   6. Приёмник: ошибка кадра (стоп-бит = 0) и последующее восстановление
//   7. Шлейф TX -> RX
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module UART_tb;

    localparam int  CLK_FREQ     = 50_000_000;
    localparam int  BAUD         = 115_200;
    localparam int  CLKS_PER_BIT = (CLK_FREQ + BAUD / 2) / BAUD;  // как в UART.sv
    localparam real CLK_NS       = 1.0e9 / CLK_FREQ;
    localparam real BIT_NS       = CLKS_PER_BIT * CLK_NS;         // длительность бита DUT

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk   = 1'b0;
    logic       rst_n = 1'b0;

    logic [7:0] tx_data  = '0;
    logic       tx_valid = 1'b0;
    logic       tx_ready;
    logic       tx;

    logic       rx;
    logic [7:0] rx_data;
    logic       rx_valid;
    logic       rx_error;

    logic       loopback = 1'b0;  // 1 - вход RX подключён к выходу TX
    logic       line     = 1'b1;  // линия, которую формирует тестбенч

    assign rx = loopback ? tx : line;

    always #(CLK_NS / 2) clk = ~clk;

    UART #(
        .CLK_FREQ (CLK_FREQ),
        .BAUD     (BAUD)
    ) dut (.*);

    // -------------------------------------------------------------------------
    // Учёт ошибок и ожидаемых данных
    // -------------------------------------------------------------------------
    int         errors     = 0;
    logic [7:0] tx_exp[$];        // байты, которые должны появиться на линии TX
    logic [7:0] rx_exp[$];        // байты, которые должен выдать приёмник
    int         tx_frames  = 0;
    int         rx_bytes   = 0;
    int         rx_err_exp = 0;
    int         rx_err_got = 0;

    task automatic fail(input string msg);
        errors++;
        $display("[%0t] ERROR: %s", $time, msg);
    endtask

    // -------------------------------------------------------------------------
    // Монитор линии TX: декодирует кадры и проверяет моменты переключения
    // -------------------------------------------------------------------------
    logic    tx_in_frame = 1'b0;
    realtime tx_t0;

    initial begin : tx_monitor
        logic [7:0] b;
        logic [7:0] exp;
        wait (rst_n);
        forever begin
            @(negedge tx);
            tx_t0       = $realtime;
            tx_in_frame = 1'b1;

            #(BIT_NS / 2);                       // середина старт-бита
            if (tx !== 1'b0) fail("TX: start bit is not 0");
            if (tx_ready)    fail("TX: tx_ready = 1 during transmission");

            for (int i = 0; i < 8; i++) begin
                #(BIT_NS);
                b[i] = tx;
            end

            #(BIT_NS);                           // середина стоп-бита
            if (tx !== 1'b1) fail("TX: stop bit is not 1");
            tx_in_frame = 1'b0;

            tx_frames++;
            if (tx_exp.size() == 0)
                fail($sformatf("TX: unexpected frame %h", b));
            else begin
                exp = tx_exp.pop_front();
                if (b !== exp) fail($sformatf("TX: got %h, expected %h", b, exp));
            end
        end
    end

    // каждое переключение TX внутри кадра должно попадать на границу бита
    always @(tx) begin
        if (tx_in_frame && $realtime != tx_t0) begin
            automatic real k   = ($realtime - tx_t0) / BIT_NS;
            automatic real dev = (k - $rtoi(k + 0.5)) * BIT_NS;
            if (dev > CLK_NS / 2 || dev < -CLK_NS / 2)
                fail($sformatf("TX: edge off bit boundary by %0.1f ns", dev));
        end
    end

    // -------------------------------------------------------------------------
    // Монитор выхода приёмника
    // -------------------------------------------------------------------------
    logic rx_valid_d = 1'b0;
    logic rx_error_d = 1'b0;

    always @(posedge clk) begin
        rx_valid_d <= rx_valid;
        rx_error_d <= rx_error;

        if (rx_valid && rx_valid_d) fail("RX: rx_valid longer than 1 clock");
        if (rx_error && rx_error_d) fail("RX: rx_error longer than 1 clock");
        if (rx_valid && rx_error)   fail("RX: rx_valid and rx_error at the same time");

        if (rx_valid) begin
            rx_bytes++;
            if (rx_exp.size() == 0)
                fail($sformatf("RX: unexpected byte %h", rx_data));
            else begin
                automatic logic [7:0] exp = rx_exp.pop_front();
                if (rx_data !== exp) fail($sformatf("RX: got %h, expected %h", rx_data, exp));
            end
        end

        if (rx_error) rx_err_got++;
    end

    // -------------------------------------------------------------------------
    // Задачи-драйверы
    // -------------------------------------------------------------------------

    // передать байт через DUT (рукопожатие valid/ready)
    task automatic tx_send(input logic [7:0] b);
        tx_data  <= b;
        tx_valid <= 1'b1;
        @(posedge clk);
        while (!tx_ready) @(posedge clk);
        tx_valid <= 1'b0;
        tx_exp.push_back(b);
    endtask

    // сформировать кадр на линии RX
    task automatic rx_drive(input logic [7:0] b,
                            input real        bit_ns = BIT_NS,
                            input logic       stop   = 1'b1);
        line = 1'b0;                 // старт-бит
        #(bit_ns);
        for (int i = 0; i < 8; i++) begin
            line = b[i];
            #(bit_ns);
        end
        line = stop;                 // стоп-бит
        #(bit_ns);
        line = 1'b1;
    endtask

    // ожидание, пока все ожидаемые байты не будут получены
    task automatic drain();
        fork
            wait (tx_exp.size() == 0 && rx_exp.size() == 0);
            begin
                #(BIT_NS * 10 * 64);
                fail("timeout waiting for data");
            end
        join_any
        disable fork;
        #(BIT_NS * 2);
    endtask

    task automatic idle(input int bits);
        #(BIT_NS * bits);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    localparam logic [7:0] PATTERNS[8] = '{8'h00, 8'hFF, 8'h55, 8'hAA,
                                           8'h01, 8'h80, 8'h0F, 8'hF0};

    initial begin
        logic [7:0] b;

        $timeformat(-6, 1, " us", 0);
        $display("UART_tb: CLK_FREQ = %0d Hz, BAUD = %0d, CLKS_PER_BIT = %0d, bit = %0.1f ns",
                 CLK_FREQ, BAUD, CLKS_PER_BIT, BIT_NS);

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // 1 ------------------------------------------------------------------
        $display("[%0t] 1. Reset state", $time);
        if (tx !== 1'b1)       fail("after reset tx != 1");
        if (tx_ready !== 1'b1) fail("after reset tx_ready != 1");
        if (rx_valid !== 1'b0) fail("after reset rx_valid != 0");
        if (rx_error !== 1'b0) fail("after reset rx_error != 0");

        // 2 ------------------------------------------------------------------
        $display("[%0t] 2. Transmitter: patterns and back-to-back stream", $time);
        foreach (PATTERNS[i]) tx_send(PATTERNS[i]);
        for (int i = 0; i < 16; i++) tx_send($urandom_range(0, 255));
        drain();
        if (tx !== 1'b1) fail("TX: line is not idle (1) after transmission");

        // 3 ------------------------------------------------------------------
        $display("[%0t] 3. Receiver: nominal baud, frames without gaps", $time);
        foreach (PATTERNS[i]) begin
            rx_exp.push_back(PATTERNS[i]);
            rx_drive(PATTERNS[i]);
        end
        for (int i = 0; i < 16; i++) begin
            b = $urandom_range(0, 255);
            rx_exp.push_back(b);
            rx_drive(b);
        end
        drain();

        // 4 ------------------------------------------------------------------
        $display("[%0t] 4. Receiver: sender baud -3%% and +3%%", $time);
        for (int i = 0; i < 8; i++) begin
            b = $urandom_range(0, 255);
            rx_exp.push_back(b);
            rx_drive(b, BIT_NS * 1.03);      // передатчик медленнее
        end
        for (int i = 0; i < 8; i++) begin
            b = $urandom_range(0, 255);
            rx_exp.push_back(b);
            rx_drive(b, BIT_NS * 0.97);      // передатчик быстрее
        end
        drain();

        // 5 ------------------------------------------------------------------
        $display("[%0t] 5. Receiver: short glitch is ignored", $time);
        line = 1'b0;
        #(BIT_NS / 4);
        line = 1'b1;
        idle(12);                            // ни байта, ни ошибки не ожидается

        // 6 ------------------------------------------------------------------
        $display("[%0t] 6. Receiver: frame error and recovery", $time);
        rx_err_exp++;
        rx_drive(8'hA5, BIT_NS, 1'b0);       // стоп-бит = 0
        idle(2);
        rx_exp.push_back(8'h3C);
        rx_drive(8'h3C);                     // следующий кадр должен приниматься
        drain();

        // 7 ------------------------------------------------------------------
        $display("[%0t] 7. Loopback TX -> RX", $time);
        loopback = 1'b1;
        for (int i = 0; i < 32; i++) begin
            b = $urandom_range(0, 255);
            rx_exp.push_back(b);
            tx_send(b);
        end
        drain();
        loopback = 1'b0;

        // Итог ---------------------------------------------------------------
        if (rx_err_got != rx_err_exp)
            fail($sformatf("RX: frame errors %0d, expected %0d", rx_err_got, rx_err_exp));

        $display("------------------------------------------------------------");
        $display("TX frames: %0d, RX bytes: %0d, RX frame errors: %0d, ERRORS: %0d",
                 tx_frames, rx_bytes, rx_err_got, errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    // защита от зависания
    initial begin
        #100ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
