// =============================================================================
// Тестбенч для spi_slave (режим 0, тактовая частота 50 МГц)
//
// Тестбенч играет роль мастера: побитно двигает CS_N, SCK, MOSI с задержками
// в наносекундах, не кратными такту clk (фаза SCK относительно clk случайна),
// и читает MISO перед фронтом SCK. Источник байтов ответа (valid/ready) начинает
// выдавать их через заданное число тактов после frame_start.
// Классы:
//   Scoreboard - ожидания по транзакции и сравнение: принятые байты, байты MISO,
//                число импульсов frame_start / frame_end / frame_short / tx_underrun.
// Проверки:
//   1. После сброса: MISO в Z (spi_miso_oe = 0), tx_ready = 0, событий нет
//   2. Кадр из 4 байт: rx_data = MOSI, MISO = ответ, по одному frame_start / frame_end
//   3. Байты 0x00, 0xFF, 0x80, 0x01 и случайные
//   4. Источник отвечает только на 2 байта из 4: остальные байты 0xFF, tx_underrun
//   5. Первый байт ответа приходит через 5, 30 тактов: ответ верный (до первого фронта SCK)
//   6. Первый байт ответа приходит слишком поздно (100 тактов): tx_underrun, байт 0xFF
//   7. CS_N поднят посреди байта (3 бита): frame_short, rx_valid нет; следующий кадр верный
//   8. Помехи на SCK и MOSI при CS_N = 1: ничего не принимается, следующий кадр верный
//   9. Паузы между байтами (SCK = 0, CS_N = 0), SCK 3 МГц
//  10. 300 случайных кадров по 1..6 байт, SCK 0.5..2 МГц
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module spi_slave_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;

    typedef logic [7:0] bytes_t[$];

    class Scoreboard;
        int errors = 0;

        function void check(string name, bytes_t mosi, bytes_t resp, int clocked_bytes,
                            bytes_t rx_log, bytes_t miso_log,
                            int n_start, int n_end, int n_short, int n_under,
                            bit want_short, bit under_any);
            int full = want_short ? clocked_bytes - 1 : clocked_bytes;   // целые байты
            int exp_under = (clocked_bytes > resp.size()) ? clocked_bytes - resp.size() : 0;
            if (rx_log.size() != full) begin
                err($sformatf("%s: rx_valid pulses %0d, expected %0d", name, rx_log.size(), full));
            end else begin
                for (int i = 0; i < full; i++)
                    if (rx_log[i] !== mosi[i])
                        err($sformatf("%s: rx byte %0d = %h, expected %h", name, i, rx_log[i], mosi[i]));
            end
            if (miso_log.size() != full) begin
                err($sformatf("%s: MISO bytes %0d, expected %0d", name, miso_log.size(), full));
            end else if (!under_any) begin
                for (int i = 0; i < full; i++) begin
                    logic [7:0] e = (i < resp.size()) ? resp[i] : 8'hFF;
                    if (miso_log[i] !== e)
                        err($sformatf("%s: MISO byte %0d = %h, expected %h", name, i, miso_log[i], e));
                end
            end
            if (n_start != 1 || n_end != 1)
                err($sformatf("%s: frame_start %0d, frame_end %0d (expected 1, 1)", name, n_start, n_end));
            if (n_short != (want_short ? 1 : 0))
                err($sformatf("%s: frame_short %0d, expected %0d", name, n_short, want_short ? 1 : 0));
            if (under_any) begin
                if (n_under == 0) err($sformatf("%s: tx_underrun expected, got none", name));
            end else if (n_under != exp_under) begin
                err($sformatf("%s: tx_underrun %0d, expected %0d", name, n_under, exp_under));
            end
        endfunction

        function void err(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk   = 1'b0;
    logic       rst_n = 1'b0;
    logic       spi_cs_n = 1'b1;
    logic       spi_sck  = 1'b0;
    logic       spi_mosi = 1'b0;
    logic       spi_miso, spi_miso_oe;
    logic [7:0] rx_data;
    logic       rx_valid;
    logic [7:0] tx_data  = '0;
    logic       tx_valid = 1'b0;
    logic       tx_ready, tx_underrun;
    logic       frame_start, frame_end, frame_short;

    always #(CLK_NS / 2) clk = ~clk;

    spi_slave dut (.*);

    Scoreboard sb = new();

    // -------------------------------------------------------------------------
    // Источник байтов ответа (valid/ready)
    // -------------------------------------------------------------------------
    bytes_t resp;
    int     first_delay = 5;        // тактов от frame_start до первого байта
    int     f_idx, f_wait;

    always @(posedge clk) begin
        if (frame_start) begin
            f_idx    = 0;
            f_wait   = first_delay;
            tx_valid <= 1'b0;
        end else if (tx_valid && tx_ready) begin
            tx_valid <= 1'b0;
            f_idx++;
        end else if (!tx_valid && f_idx < resp.size()) begin
            if (f_wait > 0) begin
                f_wait--;
            end else begin
                tx_data  <= resp[f_idx];
                tx_valid <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Журналы событий
    // -------------------------------------------------------------------------
    bytes_t rx_log, miso_log;
    int     n_start, n_end, n_short, n_under;

    always @(posedge clk) begin
        if (rx_valid)    rx_log.push_back(rx_data);
        if (frame_start) n_start++;
        if (frame_end)   n_end++;
        if (frame_short) n_short++;
        if (tx_underrun) n_under++;
    end

    // линия MISO: за пределами кадра слейв её не держит
    wire miso_line = spi_miso_oe ? spi_miso : 1'bz;

    // -------------------------------------------------------------------------
    // Мастер: побитовая передача кадра
    //   half_ns - полупериод SCK; nbits - сколько бит передать (-1: все);
    //   pause_ns - максимальная случайная пауза между байтами (SCK = 0)
    // -------------------------------------------------------------------------
    task automatic clear_logs();
        rx_log.delete();
        miso_log.delete();
        n_start = 0; n_end = 0; n_short = 0; n_under = 0;
    endtask

    task automatic xfer(input bytes_t mosi, input real half_ns = 500.0,
                        input int nbits = -1, input int pause_ns = 0);
        int total = (nbits < 0) ? 8 * mosi.size() : nbits;
        logic [7:0] m_sample;
        spi_cs_n = 1'b0;
        spi_mosi = mosi[0][7];
        #(1100.0 + $urandom_range(0, 400));              // от спада CS_N до первого фронта SCK
        for (int k = 0; k < total; k++) begin
            #(half_ns + $urandom_range(0, 9));
            m_sample = {m_sample[6:0], (miso_line === 1'b1)};
            if (k % 8 == 7) miso_log.push_back(m_sample);
            spi_sck = 1'b1;
            #(half_ns + $urandom_range(0, 9));
            spi_sck = 1'b0;
            if (k + 1 < total) begin
                if (k % 8 == 7 && pause_ns > 0) #($urandom_range(0, pause_ns));
                spi_mosi = mosi[(k + 1) / 8][7 - ((k + 1) % 8)];
            end
        end
        #(half_ns);
        spi_cs_n = 1'b1;
        #(2500.0);
    endtask

    // кадр с проверкой
    task automatic frame(input string name, input bytes_t mosi, input bytes_t r,
                         input real half_ns = 500.0, input int nbits = -1,
                         input int pause_ns = 0, input int delay = 5, input bit under_any = 0);
        bit want_short = (nbits >= 0) && (nbits % 8 != 0);
        int clocked    = (nbits < 0) ? mosi.size() : (nbits + 7) / 8;
        clear_logs();
        resp        = r;
        first_delay = delay;
        xfer(mosi, half_ns, nbits, pause_ns);
        repeat (6) @(posedge clk);
        sb.check(name, mosi, r, clocked, rx_log, miso_log, n_start, n_end, n_short, n_under,
                 want_short, under_any);
    endtask

    // MISO должен быть в Z, пока CS_N = 1
    int cs_high_clks = 0, cs_low_clks = 0;

    always @(posedge clk) begin
        if (rst_n && spi_cs_n && spi_miso_oe !== 1'b0) begin
            // допускаем задержку синхронизатора: проверяем только если CS_N высок давно
            if (cs_high_clks > 5) sb.err("spi_miso_oe = 1 while CS_N = 1");
        end
        if (rst_n && !spi_cs_n && cs_low_clks > 5 && spi_miso_oe !== 1'b1)
            sb.err("spi_miso_oe = 0 while CS_N = 0");
        if (rst_n && spi_cs_n && cs_high_clks > 5 && tx_ready !== 1'b0)
            sb.err("tx_ready = 1 outside a frame");
    end

    always @(posedge clk) begin
        cs_high_clks <= spi_cs_n ? cs_high_clks + 1 : 0;
        cs_low_clks  <= spi_cs_n ? 0 : cs_low_clks + 1;
    end

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        $display("[%0t] 1. Reset state", $time);
        if (spi_miso_oe !== 1'b0 || tx_ready !== 1'b0 || rx_valid !== 1'b0)
            sb.err($sformatf("after reset oe=%b tx_ready=%b rx_valid=%b", spi_miso_oe, tx_ready, rx_valid));

        $display("[%0t] 2. 4-byte frame", $time);
        frame("frame4", '{8'hA2, 8'h00, 8'h00, 8'h00}, '{8'hC5, 8'h12, 8'h34, 8'h7F});

        $display("[%0t] 3. Corner byte values", $time);
        frame("corner", '{8'h00, 8'hFF, 8'h80, 8'h01}, '{8'hFF, 8'h00, 8'h01, 8'h80});
        frame("alt",    '{8'h55, 8'hAA, 8'h0F, 8'hF0}, '{8'hAA, 8'h55, 8'hF0, 8'h0F});

        $display("[%0t] 4. Source answers only 2 bytes of 4", $time);
        frame("short_resp", '{8'h01, 8'h02, 8'h03, 8'h04}, '{8'h11, 8'h22});
        frame("no_resp",    '{8'h01, 8'h02}, '{});

        $display("[%0t] 5. First response byte after 5 and 30 clocks", $time);
        frame("delay5",  '{8'h00, 8'h00, 8'h00, 8'h00}, '{8'h81, 8'h82, 8'h83, 8'h84}, 500.0, -1, 0, 5);
        frame("delay30", '{8'h00, 8'h00, 8'h00, 8'h00}, '{8'h91, 8'h92, 8'h93, 8'h94}, 500.0, -1, 0, 30);

        $display("[%0t] 6. First response byte too late", $time);
        frame("late", '{8'h00, 8'h00}, '{8'hA1, 8'hA2}, 500.0, -1, 0, 100, 1);

        $display("[%0t] 7. CS_N raised in the middle of a byte", $time);
        frame("short3", '{8'hA1, 8'hA2}, '{8'h55, 8'h66}, 500.0, 3);
        frame("short11", '{8'hA1, 8'hA2}, '{8'h55, 8'h66}, 500.0, 11);
        frame("after_short", '{8'hC3, 8'h3C, 8'hDE, 8'hAD}, '{8'h12, 8'h34, 8'h56, 8'h78});

        $display("[%0t] 8. Noise on SCK and MOSI while CS_N = 1", $time);
        clear_logs();
        resp = '{8'h77};
        repeat (20) begin
            #($urandom_range(30, 400)) spi_sck = ~spi_sck;
            #($urandom_range(30, 400)) spi_mosi = $urandom;
        end
        spi_sck = 1'b0;
        #1000;
        if (rx_log.size() != 0 || n_start != 0 || n_end != 0 || n_short != 0 || n_under != 0)
            sb.err("noise at CS_N = 1 caused events");
        frame("after_noise", '{8'h5A, 8'hA5, 8'h3C, 8'hC3}, '{8'h01, 8'h23, 8'h45, 8'h67});

        $display("[%0t] 9. Pauses between bytes, SCK 3 MHz", $time);
        frame("pauses", '{8'h12, 8'h34, 8'h56, 8'h78}, '{8'h9A, 8'hBC, 8'hDE, 8'hF0}, 500.0, -1, 250);
        frame("fast",   '{8'h12, 8'h34, 8'h56, 8'h78}, '{8'h9A, 8'hBC, 8'hDE, 8'hF0}, 167.0);

        $display("[%0t] 10. 300 random frames", $time);
        for (int k = 0; k < 300; k++) begin
            automatic int n = $urandom_range(1, 6);
            bytes_t d, r;
            d.delete();
            r.delete();
            repeat (n) begin
                d.push_back($urandom);
                r.push_back($urandom);
            end
            frame($sformatf("rand%0d", k), d, r, 250.0 + $urandom_range(0, 750), -1,
                  $urandom_range(0, 1) ? $urandom_range(0, 300) : 0, $urandom_range(3, 40));
        end

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d", sb.errors);
        if (sb.errors == 0) $display("TEST PASSED");
        else                $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #30ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
