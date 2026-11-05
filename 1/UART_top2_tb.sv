// =============================================================================
// Тестбенч для UART_top2: парсеры и конвейер вместе, через линии UART
//
// Тестбенч играет роль МУ: передаёт байты на линию uart_rx побитно и
// принимает линию uart_tx. Параметры уменьшены: 1 Мбит/с (50 тактов на бит),
// период генератора 6000 тактов, тайм-аут пакета 1500 тактов (3 байта),
// антидребезг 20 тактов, слот индикатора 20 тактов.
//
// Классы:
//   Packet    - пакет МО -> МУ: поля, байты на линии, сравнение;
//   PktStream - разбор принятого потока как в МУ: поиск 0xAA, проверка CHK,
//               при ошибке сдвиг на 1 байт; байты вне пакетов копятся как текст;
//   GenModel  - эталон генератора test_pkt_gen (значения пакета номер n);
//   Board     - ожидаемые показания счётчиков индикации.
// Проверки:
//   1. После сброса: линия TX = 1, светодиоды погашены
//   2. Мусор (в том числе 0x03, 0xFF, 0x55) - ничего не передаётся, счётчик u
//   3. Старт 0x01: пакеты генератора с n = 0, период; Стоп 0x02: пакеты прекращаются
//   4. Эхо 20 верных пакетов вплотную; 0x01/0x02 внутри пакета - не команды
//   5. Пакеты с неверной CHK - эха нет, счётчик ошибок
//   6. Оборванный пакет и пауза - ошибка, следующий пакет принимается
//   7. KEY1 - один пакет генератора, KEY2 - строка "UART_top2\r\n"
//   8. Старт + эхо одновременно: пакеты не перемешиваются
//   8b. МУ передаёт на 1 % быстрее ПЛИС: 25 пакетов вплотную, эхо не теряется
//      (очередь эха; без неё терялся бы примерно каждый 14-й пакет, как на плате)
//   9. Счётчики индикации r, t, E, u совпадают с ожидаемыми; KEY4 - режим,
//      KEY3 - обнуление
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module UART_top2_tb;

    localparam int  CLK_FREQ   = 50_000_000;
    localparam int  BAUD       = 1_000_000;
    localparam int  CLKS_BIT   = (CLK_FREQ + BAUD / 2) / BAUD;
    localparam real CLK_NS     = 1.0e9 / CLK_FREQ;
    localparam real BIT_NS     = CLKS_BIT * CLK_NS;
    localparam int  PERIOD     = 6000;              // период генератора, тактов
    localparam int  TIMEOUT    = 30 * CLKS_BIT;     // обрыв пакета: 3 байта
    localparam int  DEB        = 20;
    localparam int  PKT_CLKS   = 80 * CLKS_BIT;     // время передачи пакета
    localparam string MSG      = "UART_top2\015\012";   // "UART_top2\r\n"

    typedef logic [7:0] bytes_t[$];

    // -------------------------------------------------------------------------
    // Классы
    // -------------------------------------------------------------------------
    class Packet;
        logic [15:0] meas, filt, err;

        function new(logic [15:0] m = $urandom, logic [15:0] f = $urandom,
                     logic [15:0] e = $urandom);
            meas = m;
            filt = f;
            err  = e;
        endfunction

        static function Packet make(logic [15:0] m, logic [15:0] f, logic [15:0] e);
            Packet p = new(m, f, e);
            return p;
        endfunction

        function logic [7:0] chk();
            return meas[15:8] ^ meas[7:0] ^ filt[15:8] ^ filt[7:0] ^ err[15:8] ^ err[7:0];
        endfunction

        // байты на линии; chk_flip != 0 портит контрольную сумму
        function bytes_t to_bytes(logic [7:0] chk_flip = 8'h00);
            return '{8'hAA, meas[15:8], meas[7:0], filt[15:8], filt[7:0],
                     err[15:8], err[7:0], chk() ^ chk_flip};
        endfunction

        function bit same(Packet p);
            return {meas, filt, err} === {p.meas, p.filt, p.err};
        endfunction

        function string str();
            return $sformatf("%h %h %h", meas, filt, err);
        endfunction
    endclass

    class PktStream;
        logic [7:0] buffer[$];
        Packet      packets[$];         // разобранные пакеты по порядку
        string      text = "";          // байты вне пакетов
        int         n_bad = 0;          // сдвигов из-за неверной CHK

        function void push(logic [7:0] b);
            buffer.push_back(b);
            while (buffer.size() != 0) begin
                if (buffer[0] != 8'hAA) begin
                    text = {text, string'(buffer.pop_front())};
                end else if (buffer.size() < 8) begin
                    break;
                end else begin
                    automatic Packet p = Packet::make({buffer[1], buffer[2]},
                                                      {buffer[3], buffer[4]},
                                                      {buffer[5], buffer[6]});
                    if (buffer[7] == p.chk()) begin
                        packets.push_back(p);
                        repeat (8) void'(buffer.pop_front());
                    end else begin
                        n_bad++;
                        text = {text, string'(buffer.pop_front())};
                    end
                end
            end
        endfunction
    endclass

    class GenModel;
        int n = 0;

        function Packet next();
            logic [15:0] m = 16'h1900 + 16'(n) * 16'h0020;
            logic [15:0] f = 16'h1900 + 16'(n) * 16'h0010;
            int          d = int'($signed(m)) - int'($signed(f));
            logic [15:0] e = (d > 32767) ? 16'h7FFF : (d < -32768) ? 16'h8000 : 16'(d);
            n++;
            return Packet::make(m, f, e);
        endfunction
    endclass

    class Board;
        int rx_pkt = 0, tx_pkt = 0, err = 0, garbage = 0;
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk     = 1'b0;
    logic       rst_n   = 1'b0;
    logic [3:0] key     = 4'hF;
    logic       uart_rx = 1'b1;
    logic       uart_tx;
    logic [7:0] seg;
    logic [3:0] dig;
    logic [3:0] led;

    always #(CLK_NS / 2) clk = ~clk;

    UART_top2 #(
        .CLK_FREQ        (CLK_FREQ),
        .BAUD            (BAUD),
        .PKT_PERIOD_CLKS (PERIOD),
        .RX_TIMEOUT_CLKS (TIMEOUT),
        .DEB_CLKS        (DEB),
        .DIGIT_HZ        (CLK_FREQ / 20),
        .BLANK_CLKS      (2),
        .FLASH_CLKS      (100)
    ) dut (.*);

    wire [3:0] led_on = ~led;

    PktStream rx_stream = new();        // всё, что ПЛИС передала
    GenModel  gen       = new();
    Board     board     = new();
    Packet    echo_exp[$];              // ожидаемые эхо-пакеты по порядку
    int       errors    = 0;
    int       n_echo    = 0, n_gen = 0;
    realtime  last_gen_t = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 30) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    // -------------------------------------------------------------------------
    // Приём линии uart_tx (сторона МУ)
    // -------------------------------------------------------------------------
    initial begin : tx_monitor
        logic [7:0] b;
        wait (rst_n);
        forever begin
            @(negedge uart_tx);
            #(BIT_NS / 2);
            if (uart_tx !== 1'b0) begin fail("TX: false start bit"); continue; end
            for (int i = 0; i < 8; i++) begin
                #(BIT_NS);
                b[i] = uart_tx;
            end
            #(BIT_NS);
            if (uart_tx !== 1'b1) fail("TX: stop bit is not 1");
            rx_stream.push(b);
        end
    end

    // разбор принятых пакетов: эхо (если совпадает с ожидаемым) или генератор
    initial begin : pkt_checker
        Packet p, g;
        forever begin
            @(posedge clk);
            if (rx_stream.packets.size() == 0) continue;
            p = rx_stream.packets.pop_front();
            if (echo_exp.size() != 0 && p.same(echo_exp[0])) begin
                void'(echo_exp.pop_front());
                n_echo++;
            end else begin
                g = gen.next();
                if (!p.same(g))
                    fail({"unexpected packet ", p.str(), ", generator expected ", g.str(),
                          (echo_exp.size() ? {", echo expected ", echo_exp[0].str()} : "")});
                n_gen++;
                last_gen_t = $realtime;
            end
            board.tx_pkt++;
        end
    end

    // -------------------------------------------------------------------------
    // Передача на линию uart_rx (сторона МУ)
    // -------------------------------------------------------------------------
    real host_bit_ns = BIT_NS;          // длительность бита у МУ (может отличаться от ПЛИС)

    task automatic send_byte(input logic [7:0] b);
        uart_rx = 1'b0;
        #(host_bit_ns);
        for (int i = 0; i < 8; i++) begin
            uart_rx = b[i];
            #(host_bit_ns);
        end
        uart_rx = 1'b1;
        #(host_bit_ns);
    endtask

    task automatic send_bytes(input bytes_t b);
        foreach (b[i]) send_byte(b[i]);
    endtask

    // верный пакет: ждём эхо
    task automatic send_good(input Packet p);
        echo_exp.push_back(p);
        board.rx_pkt++;
        send_bytes(p.to_bytes());
    endtask

    task automatic press(input int k);
        key[k] = 1'b0;
        repeat (3 * DEB) @(posedge clk);
        key[k] = 1'b1;
        repeat (3 * DEB) @(posedge clk);
    endtask

    task automatic wait_clks(input int n);
        repeat (n) @(posedge clk);
    endtask

    // ожидание, пока все ожидаемые эхо-пакеты придут
    task automatic wait_echo();
        int t = 0;
        while (echo_exp.size() != 0 && t < 20 * PKT_CLKS) begin
            @(posedge clk);
            t++;
        end
        if (echo_exp.size() != 0) fail($sformatf("%0d echo packets missing, first %s",
                                                 echo_exp.size(), echo_exp[0].str()));
        wait_clks(PKT_CLKS);
    endtask

    task automatic check_quiet(input string what, input int clks);
        int n0 = n_echo + n_gen;
        int t0 = rx_stream.text.len();
        wait_clks(clks);
        if (n_echo + n_gen != n0 || rx_stream.text.len() != t0)
            fail({"FPGA transmitted something: ", what});
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        Packet p;
        int    n0;

        $timeformat(-6, 1, " us", 0);
        $display("UART_top2_tb: BAUD = %0d, %0d clocks per bit, generator period %0d clocks",
                 BAUD, CLKS_BIT, PERIOD);
        wait_clks(5);
        rst_n = 1'b1;
        wait_clks(5);

        // 1 ------------------------------------------------------------------
        $display("[%0t] 1. Reset state", $time);
        if (uart_tx !== 1'b1) fail("after reset uart_tx != 1");
        if (led_on  !== 4'b0) fail($sformatf("after reset led = %b", led_on));

        // 2 ------------------------------------------------------------------
        $display("[%0t] 2. Garbage is ignored", $time);
        send_bytes('{8'h00, 8'h03, 8'hFF, 8'h55, 8'h10, 8'h20});
        board.garbage += 6;
        check_quiet("after garbage", 2 * PERIOD);
        if (led_on[2]) fail("running after garbage");

        // 3 ------------------------------------------------------------------
        $display("[%0t] 3. Start: generator packets, Stop", $time);
        gen.n = 0;
        send_byte(8'h01);
        wait_clks(10);
        if (!led_on[2]) fail("led[2] (running) is off after Start");
        wait_clks(4 * PERIOD + PKT_CLKS);
        if (n_gen != 5) fail($sformatf("%0d generator packets after Start, expected 5", n_gen));
        send_byte(8'h02);
        wait_clks(PKT_CLKS + 10);               // пакет, начатый до Стоп, допускается
        if (led_on[2]) fail("led[2] (running) is on after Stop");
        n0 = n_gen;
        check_quiet("after Stop", 3 * PERIOD);

        // 4 ------------------------------------------------------------------
        $display("[%0t] 4. Echo of 20 good packets back-to-back", $time);
        send_good(Packet::make(16'h0102, 16'h0201, 16'h0102));    // команды внутри пакета
        send_good(Packet::make(16'hAAAA, 16'hAAAA, 16'hAAAA));
        send_good(Packet::make(16'h1960, 16'h1900, 16'h0060));
        for (int i = 0; i < 17; i++) begin
            p = new();
            send_good(p);
        end
        wait_echo();
        if (n_gen != n0) fail("generator started by 0x01 inside a packet");
        if (led_on[2])  fail("running after packets with 0x01 inside");

        // 5 ------------------------------------------------------------------
        $display("[%0t] 5. Packets with wrong CHK: no echo", $time);
        for (int i = 0; i < 3; i++) begin
            p = new();
            send_bytes(p.to_bytes(8'(1 << i)));
            board.err++;
        end
        check_quiet("after bad packets", 2 * PKT_CLKS);

        // 6 ------------------------------------------------------------------
        $display("[%0t] 6. Cut packet and pause, then a good packet", $time);
        begin
            bytes_t b;
            p = new();
            b = p.to_bytes();
            send_bytes(b[0:3]);
            board.err++;
        end
        wait_clks(2 * TIMEOUT);
        p = new();
        send_good(p);
        wait_echo();

        // 7 ------------------------------------------------------------------
        $display("[%0t] 7. KEY1: one generator packet, KEY2: message", $time);
        n0 = n_gen;
        press(0);
        wait_clks(PKT_CLKS + 100);
        if (n_gen != n0 + 1) fail($sformatf("KEY1: %0d packets, expected 1", n_gen - n0));
        rx_stream.text = "";
        press(1);
        wait_clks(MSG.len() * 10 * CLKS_BIT + 100);
        if (rx_stream.text != MSG) fail($sformatf("KEY2: got \"%s\"", rx_stream.text));

        // 8 ------------------------------------------------------------------
        $display("[%0t] 8. Start + echo at the same time", $time);
        gen.n = 0;
        n0    = n_gen;
        send_byte(8'h01);
        for (int i = 0; i < 15; i++) begin
            p = new();
            send_good(p);
        end
        wait_echo();
        send_byte(8'h02);
        wait_clks(PKT_CLKS + 100);
        if (n_gen - n0 < 3) fail($sformatf("only %0d generator packets during echo", n_gen - n0));

        // 8b -----------------------------------------------------------------
        $display("[%0t] 8b. MU is 1%% faster: 25 packets back-to-back, no echo lost", $time);
        host_bit_ns = BIT_NS * 0.99;
        for (int i = 0; i < 25; i++) begin
            p = new();
            send_good(p);
        end
        wait_echo();
        host_bit_ns = BIT_NS;

        // 9 ------------------------------------------------------------------
        $display("[%0t] 9. Indicator counters", $time);
        // счётчики BCD: сравниваем с ожидаемыми десятичными значениями
        if (dut.u_ind.cnt_rx_pkt  !== 12'(bcd(board.rx_pkt)))  fail($sformatf("r = %h, expected %0d", dut.u_ind.cnt_rx_pkt,  board.rx_pkt));
        if (dut.u_ind.cnt_tx_pkt  !== 12'(bcd(board.tx_pkt)))  fail($sformatf("t = %h, expected %0d", dut.u_ind.cnt_tx_pkt,  board.tx_pkt));
        if (dut.u_ind.cnt_err     !== 12'(bcd(board.err)))     fail($sformatf("E = %h, expected %0d", dut.u_ind.cnt_err,     board.err));
        if (dut.u_ind.cnt_garbage !== 12'(bcd(board.garbage))) fail($sformatf("u = %h, expected %0d", dut.u_ind.cnt_garbage, board.garbage));
        if (!led_on[3]) fail("led[3] (error) is off");
        $display("[%0t]    r %h, t %h, E %h, u %h", $time, dut.u_ind.cnt_rx_pkt, dut.u_ind.cnt_tx_pkt,
                 dut.u_ind.cnt_err, dut.u_ind.cnt_garbage);
        press(3);
        if (dut.u_ind.mode !== 1) fail("KEY4 did not switch the indicator mode");
        press(2);
        if (dut.u_ind.cnt_rx_pkt !== 0 || dut.u_ind.cnt_err !== 0 || led_on[3])
            fail("KEY3 did not clear counters / led[3]");

        if (rx_stream.n_bad != 0) fail($sformatf("%0d packets from FPGA with wrong CHK", rx_stream.n_bad));
        if (rx_stream.text != MSG) fail("FPGA sent bytes outside packets");

        $display("------------------------------------------------------------");
        $display("echo packets: %0d, generator packets: %0d, ERRORS: %0d", n_echo, n_gen, errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    function automatic int bcd(input int v);
        return ((v / 100) % 10) * 256 + ((v / 10) % 10) * 16 + v % 10;
    endfunction

    initial begin
        #100ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
