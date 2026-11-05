// =============================================================================
// Тестбенч для uart_pkt_parser (парсер пакетов 0xAA + 6 байт + XOR)
//
// Классы:
//   Event      - событие на выходе парсера: пакет, ошибка CHK, обрыв, байт вне пакета;
//   StreamGen  - генератор входного потока: верные пакеты, пакеты с неверной CHK,
//                мусор, оборванные пакеты; для каждого куска кладёт в табло
//                ожидаемые события;
//   Scoreboard - сравнение событий DUT с ожидаемыми по порядку.
// Проверки:
//   1. После сброса: нет импульсов, busy = 0
//   2. Граничные случаи: 0xAA внутри данных, CHK = 0xAA, пакет вплотную к пакету,
//      мусор между пакетами
//   3. Обрыв пакета: пауза TIMEOUT-1 тактов ещё допустима, TIMEOUT тактов - обрыв
//   4. Случайный поток из 600 кусков, байты с паузами 0..5 тактов
//   Всё время: импульсы не совпадают, каждый пакет/ошибка CHK/байт вне пакета
//   выдаётся через 1 такт после входного байта (лишние импульсы ловит табло).
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module uart_pkt_parser_tb;

    localparam real CLK_NS  = 20.0;
    localparam int  TIMEOUT = 40;           // уменьшенный тайм-аут, тактов
    localparam int  N_RAND  = 600;

    typedef logic [7:0] bytes_t[$];

    // -------------------------------------------------------------------------
    // Классы
    // -------------------------------------------------------------------------
    typedef enum {EV_PKT, EV_CHK_ERR, EV_TIMEOUT, EV_PASS} ev_kind_t;

    class Event;
        ev_kind_t    kind;
        logic [47:0] value;     // {meas, filt, err} для EV_PKT, байт для EV_PASS

        function new(ev_kind_t k, logic [47:0] v = '0);
            kind  = k;
            value = v;
        endfunction

        // ModelSim 10.5 не принимает new(...) аргументом вызова, поэтому фабрика
        static function Event make(ev_kind_t k, logic [47:0] v = '0);
            Event e = new(k, v);
            return e;
        endfunction

        function bit same(Event e);
            return kind == e.kind && value === e.value;
        endfunction

        function string str();
            case (kind)
                EV_PKT:     return $sformatf("PKT %h %h %h", value[47:32], value[31:16], value[15:0]);
                EV_CHK_ERR: return "CHK_ERROR";
                EV_TIMEOUT: return "TIMEOUT";
                default:    return $sformatf("PASS %h", value[7:0]);
            endcase
        endfunction
    endclass

    class Scoreboard;
        Event expected[$];
        int   n_checked = 0;
        int   errors    = 0;

        function void push(Event e);
            expected.push_back(e);
        endfunction

        function void check(Event got);
            Event exp;
            n_checked++;
            if (expected.size() == 0) begin
                error({"unexpected event ", got.str()});
                return;
            end
            exp = expected.pop_front();
            if (!got.same(exp))
                error({"got ", got.str(), ", expected ", exp.str()});
        endfunction

        function void error(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    class StreamGen;
        Scoreboard sb;
        int        n_pkt = 0, n_bad = 0, n_garbage = 0, n_cut = 0;

        function new(Scoreboard s);
            sb = s;
        endfunction

        // байты пакета: AA, данные, CHK (^ chk_flip - порча контрольной суммы)
        function bytes_t packet(logic [47:0] d, logic [7:0] chk_flip = 8'h00);
            bytes_t     b;
            logic [7:0] chk = '0;
            b.push_back(8'hAA);
            for (int i = 5; i >= 0; i--) begin
                b.push_back(d[8*i +: 8]);
                chk ^= d[8*i +: 8];
            end
            b.push_back(chk ^ chk_flip);
            return b;
        endfunction

        function bytes_t good(logic [47:0] d = {$urandom, $urandom});
            n_pkt++;
            sb.push(Event::make(EV_PKT, d));
            return packet(d);
        endfunction

        function bytes_t bad_chk();
            n_bad++;
            sb.push(Event::make(EV_CHK_ERR));
            return packet({$urandom, $urandom}, 8'($urandom_range(1, 255)));
        endfunction

        function bytes_t garbage();
            logic [7:0] b;
            do b = $urandom; while (b == 8'hAA);
            n_garbage++;
            sb.push(Event::make(EV_PASS, 48'(b)));
            return '{b};
        endfunction

        // начало пакета (1..7 байт); после него нужна пауза >= TIMEOUT
        function bytes_t cut();
            bytes_t b = packet({$urandom, $urandom});
            n_cut++;
            sb.push(Event::make(EV_TIMEOUT));
            return b[0 : $urandom_range(0, 6)];
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic        clk      = 1'b0;
    logic        rst_n    = 1'b0;
    logic [7:0]  rx_data  = '0;
    logic        rx_valid = 1'b0;
    logic [15:0] meas, filt, err;
    logic        pkt_valid, chk_error, timeout;
    logic [7:0]  pass_data;
    logic        pass_valid;
    logic        busy;

    always #(CLK_NS / 2) clk = ~clk;

    uart_pkt_parser #(
        .TIMEOUT_CLKS (TIMEOUT)
    ) dut (.*);

    Scoreboard sb  = new();
    StreamGen  gen = new(sb);

    // -------------------------------------------------------------------------
    // Монитор выходов
    // -------------------------------------------------------------------------
    logic rx_valid_d = 1'b0;

    always @(posedge clk) begin
        automatic logic [3:0] pulses = {pkt_valid, chk_error, timeout, pass_valid};
        if (rst_n) begin
            if ($countones(pulses) > 1)
                sb.error($sformatf("several pulses at once: %b", pulses));
            // pkt_valid, chk_error, pass_valid - ровно через такт после входного байта
            if ((pkt_valid || chk_error || pass_valid) && !rx_valid_d)
                sb.error("output pulse without an input byte one clock before");
            if (timeout && rx_valid_d)
                sb.error("timeout right after an input byte");
            if (pass_valid && busy)        sb.error("busy = 1 right after a byte outside a packet");

            if (pkt_valid)  sb.check(Event::make(EV_PKT, {meas, filt, err}));
            if (chk_error)  sb.check(Event::make(EV_CHK_ERR));
            if (timeout)    sb.check(Event::make(EV_TIMEOUT));
            if (pass_valid) sb.check(Event::make(EV_PASS, 48'(pass_data)));
        end
        rx_valid_d <= rx_valid;
    end

    // -------------------------------------------------------------------------
    // Драйвер: байт = импульс rx_valid, между байтами gap тактов
    // -------------------------------------------------------------------------
    task automatic drive(bytes_t b, int min_gap = 0, int max_gap = 0);
        foreach (b[i]) begin
            rx_data  <= b[i];
            rx_valid <= 1'b1;
            @(posedge clk);
            rx_valid <= 1'b0;
            rx_data  <= 'x;
            repeat ($urandom_range(min_gap, max_gap)) @(posedge clk);
        end
    endtask

    task automatic pause(int clks);
        repeat (clks) @(posedge clk);
    endtask

    task automatic drain();
        pause(TIMEOUT + 5);
        if (sb.expected.size() != 0)
            sb.error($sformatf("%0d expected events did not happen, first: %s",
                               sb.expected.size(), sb.expected[0].str()));
        sb.expected.delete();
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        bytes_t b;

        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);

        // 1 ------------------------------------------------------------------
        $display("[%0t] 1. Reset state", $time);
        if (busy !== 1'b0) sb.error("after reset busy != 0");
        pause(TIMEOUT * 2);                     // в HUNT тайм-аута нет

        // 2 ------------------------------------------------------------------
        $display("[%0t] 2. Corner cases", $time);
        drive(gen.good(48'hAAAA_AAAA_AAAA));    // данные = SYNC, CHK = 0
        drive(gen.good(48'h1960_1900_0060));    // пример из PROTOCOL.md
        drive(gen.good(48'h00AA_0000_0000));    // CHK = 0xAA
        drive(gen.good(48'h0000_0000_0000));
        drive(gen.good(48'hFFFF_FFFF_FFFF));
        drive(gen.garbage());
        drive(gen.garbage());
        drive(gen.bad_chk());
        drive(gen.garbage());                   // после ошибки CHK - снова поиск
        drive(gen.good());
        // команды МУ не путаются с пакетами
        sb.push(Event::make(EV_PASS, 48'h01));
        sb.push(Event::make(EV_PASS, 48'h02));
        drive('{8'h01, 8'h02});
        drive(gen.good(48'h0102_0102_0102));    // 0x01, 0x02 внутри пакета - не команды
        drain();

        // 3 ------------------------------------------------------------------
        $display("[%0t] 3. Timeout: gap TIMEOUT-1 is ok, gap TIMEOUT aborts", $time);
        drive(gen.good(), TIMEOUT - 1, TIMEOUT - 1);
        b = gen.packet(48'h1234_5678_9ABC);
        sb.push(Event::make(EV_TIMEOUT));
        drive(b[0:2]);
        pause(TIMEOUT);                         // TIMEOUT тактов без байта - обрыв
        sb.push(Event::make(EV_PASS, 48'(b[3])));       // остаток пакета - уже вне пакета
        drive(b[3:3]);
        drain();
        drive(gen.cut());
        pause(TIMEOUT);
        drive(gen.good());                      // после обрыва приём продолжается
        drain();

        // 4 ------------------------------------------------------------------
        $display("[%0t] 4. Random stream: %0d pieces", $time, N_RAND);
        for (int i = 0; i < N_RAND; i++) begin
            case ($urandom_range(0, 9))
                0, 1, 2, 3: drive(gen.good(),    0, 5);
                4, 5:       drive(gen.bad_chk(), 0, 5);
                6, 7, 8:    drive(gen.garbage(), 0, 5);
                default: begin
                    drive(gen.cut(), 0, 5);
                    pause(TIMEOUT);
                end
            endcase
        end
        drain();

        // Итог ---------------------------------------------------------------
        $display("------------------------------------------------------------");
        $display("generated: good %0d, bad CHK %0d, garbage %0d, cut %0d",
                 gen.n_pkt, gen.n_bad, gen.n_garbage, gen.n_cut);
        $display("events checked: %0d, ERRORS: %0d", sb.n_checked, sb.errors);
        if (sb.errors == 0) $display("TEST PASSED");
        else                $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #10ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
