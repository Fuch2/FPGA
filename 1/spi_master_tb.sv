// =============================================================================
// Тестбенч для spi_master (режим 0, SCK = 1 МГц, тактовая частота 50 МГц)
//
// Тестбенч играет роль слейва: побитовая модель на фронтах и спадах SCK.
// Классы:
//   SlaveModel - модель слейва: на спаде CS_N начинает транзакцию, по фронту SCK
//                принимает бит MOSI, по спаду SCK выдаёт следующий бит MISO;
//                по окончании транзакции складывает принятые байты в список.
// Проверки (модель + мониторы времени на линиях):
//   1. После сброса: CS_N = 1, SCK = 0, tx_ready = 1, busy = 0
//   2. Один байт (0xA5), ответ 0x3C: MOSI и rx_data верны
//   3. Кадр протокола из 4 байт (A2 00 00 00) с ответом (C5 12 34 xx)
//   4. Байты 0x00, 0xFF, 0x80, 0x01 (крайние значения)
//   5. Транзакции вплотную друг к другу: пауза CS_N = 1 не менее 2 мкс
//   6. Пауза внутри транзакции (источник не успевает): CS_N остаётся 0, SCK = 0
//   7. 300 случайных транзакций по 1..6 байт со случайными паузами
//   Мониторы на каждом такте: период SCK 1 мкс, от спада CS_N до первого фронта
//   SCK не менее 1 мкс, MOSI меняется только при SCK = 0, SCK не дёргается при CS_N = 1.
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module spi_master_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam int  SCK_FREQ = 1_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam real SCK_NS   = 1.0e9 / SCK_FREQ;

    typedef logic [7:0] bytes_t[$];

    // -------------------------------------------------------------------------
    // Модель слейва
    // -------------------------------------------------------------------------
    class SlaveModel;
        bytes_t     reply;              // что слейв выдаёт по MISO в текущей транзакции
        bytes_t     got;                // что принято по MOSI в текущей транзакции
        bytes_t     done[$];            // принятые байты завершённых транзакций
        int         bad_bits[$];        // сколько лишних бит в завершённых транзакциях
        logic [7:0] rx_sh, tx_sh;
        int         bits, out_idx;

        function void load_next();
            tx_sh = (out_idx < reply.size()) ? reply[out_idx] : 8'hFF;
            out_idx++;
        endfunction

        function void begin_txn();
            got.delete();
            bits    = 0;
            out_idx = 0;
            load_next();
        endfunction

        function void sample(logic mosi);
            rx_sh = {rx_sh[6:0], mosi};
            bits++;
            if (bits == 8) begin
                got.push_back(rx_sh);
                bits = 0;
            end
        endfunction

        function void advance();
            if (bits == 0) load_next();
            else           tx_sh = tx_sh << 1;
        endfunction

        function logic miso();
            return tx_sh[7];
        endfunction

        function void end_txn();
            done.push_back(got);
            bad_bits.push_back(bits);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk   = 1'b0;
    logic       rst_n = 1'b0;
    logic [7:0] tx_data = '0;
    logic       tx_valid = 1'b0;
    logic       tx_last  = 1'b0;
    logic       tx_ready;
    logic [7:0] rx_data;
    logic       rx_valid;
    logic       busy;
    logic       spi_cs_n, spi_sck, spi_mosi;
    logic       spi_miso = 1'b1;

    always #(CLK_NS / 2) clk = ~clk;

    spi_master #(
        .CLK_FREQ (CLK_FREQ),
        .SCK_FREQ (SCK_FREQ)
    ) dut (.*);

    SlaveModel slave = new();
    int        errors = 0;

    function automatic real absr(input real x);
        return (x < 0.0) ? -x : x;
    endfunction

    function automatic bit same(input bytes_t a, input bytes_t b);
        if (a.size() != b.size()) return 0;
        foreach (a[i]) if (a[i] !== b[i]) return 0;
        return 1;
    endfunction

    task automatic fail(input string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    // модель слейва на линиях SPI
    always @(negedge spi_cs_n) begin
        slave.begin_txn();
        spi_miso = slave.miso();
    end
    always @(posedge spi_sck) if (!spi_cs_n) slave.sample(spi_mosi);
    always @(negedge spi_sck) if (!spi_cs_n) begin
        slave.advance();
        spi_miso = slave.miso();
    end
    always @(posedge spi_cs_n) begin
        if (rst_n) slave.end_txn();
        spi_miso = 1'b1;
    end

    // принятые мастером байты
    bytes_t rx_got;
    always @(posedge clk) if (rx_valid) rx_got.push_back(rx_data);

    // -------------------------------------------------------------------------
    // Мониторы времени и формы сигналов
    // -------------------------------------------------------------------------
    realtime t_cs_fall, t_cs_rise, t_rise, t_fall;
    int      rises_in_txn = 0;
    int      n_bytes_seen = 0;

    always @(negedge spi_cs_n) begin
        if (rst_n && t_cs_rise != 0 && $realtime - t_cs_rise < 2000.0 - 0.001)
            fail($sformatf("CS_N high gap %0.1f ns < 2000 ns", $realtime - t_cs_rise));
        t_cs_fall    = $realtime;
        rises_in_txn = 0;
    end

    always @(posedge spi_sck) begin
        if (spi_cs_n) fail("SCK rising while CS_N = 1");
        if (rises_in_txn == 0 && $realtime - t_cs_fall < 1000.0)
            fail($sformatf("CS_N fall -> first SCK edge %0.1f ns < 1000 ns", $realtime - t_cs_fall));
        // внутри байта период SCK ровно 1 / SCK_FREQ
        if (rises_in_txn % 8 != 0 && absr($realtime - t_rise - SCK_NS) > 0.001)
            fail($sformatf("SCK period %0.1f ns, expected %0.1f", $realtime - t_rise, SCK_NS));
        if (rises_in_txn % 8 == 0 && rises_in_txn != 0 && $realtime - t_rise < SCK_NS - 0.001)
            fail("SCK period between bytes is shorter than one period");
        t_rise = $realtime;
        rises_in_txn++;
    end

    always @(negedge spi_sck) if (rst_n) begin
        if (absr($realtime - t_rise - SCK_NS / 2) > 0.001)
            fail($sformatf("SCK high time %0.1f ns", $realtime - t_rise));
        t_fall = $realtime;
    end

    always @(posedge spi_cs_n) begin
        if (rst_n) begin
            if (spi_sck) fail("CS_N rising while SCK = 1");
            if ($realtime - t_fall < SCK_NS / 2 - 0.001)
                fail($sformatf("SCK fall -> CS_N rise %0.1f ns", $realtime - t_fall));
        end
        t_cs_rise = $realtime;
    end

    // MOSI может меняться только при SCK = 0 (спад SCK и смена MOSI - одно и то же время)
    always @(spi_mosi) if (!spi_cs_n && spi_sck) fail("MOSI changed while SCK = 1");

    // -------------------------------------------------------------------------
    // Передача транзакции с проверкой
    // -------------------------------------------------------------------------
    int n_txn = 0;

    // d - байты на MOSI, r - байты ответа слейва; max_gap - случайные паузы
    // между байтами (тактов), stall - одна длинная пауза перед байтом stall_at
    task automatic txn(input bytes_t d, input bytes_t r, input int max_gap = 0,
                       input int stall_at = -1, input int stall_clks = 0);
        int base = slave.done.size();
        slave.reply = r;
        rx_got.delete();
        foreach (d[i]) begin
            if (i == stall_at) repeat (stall_clks) @(posedge clk);
            repeat ($urandom_range(0, max_gap)) @(posedge clk);
            tx_data  <= d[i];
            tx_valid <= 1'b1;
            tx_last  <= (i == d.size() - 1);
            do @(posedge clk); while (!tx_ready);
            tx_valid <= 1'b0;
            tx_data  <= 'x;
            tx_last  <= 1'b0;
        end
        repeat (2) @(posedge clk);
        wait (!busy);
        repeat (2) @(posedge clk);
        n_txn++;

        if (slave.done.size() != base + 1) begin
            fail($sformatf("transactions seen by slave: %0d, expected %0d", slave.done.size() - base, 1));
        end else begin
            if (!same(slave.done[base], d))
                fail($sformatf("MOSI bytes %p, expected %p", slave.done[base], d));
            if (slave.bad_bits[base] != 0)
                fail($sformatf("%0d stray bits at the end of the transaction", slave.bad_bits[base]));
        end
        // ответ слейва: принятые байты = ответ, а недостающие дополнены 0xFF не бывают
        if (rx_got.size() != d.size())
            fail($sformatf("rx_valid pulses %0d, expected %0d", rx_got.size(), d.size()));
        else foreach (d[i])
            if (rx_got[i] !== ((i < r.size()) ? r[i] : 8'hFF))
                fail($sformatf("rx byte %0d = %h, expected %h", i, rx_got[i],
                               (i < r.size()) ? r[i] : 8'hFF));
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
        if (spi_cs_n !== 1'b1 || spi_sck !== 1'b0 || tx_ready !== 1'b1 || busy !== 1'b0)
            fail($sformatf("after reset cs_n=%b sck=%b tx_ready=%b busy=%b",
                           spi_cs_n, spi_sck, tx_ready, busy));

        $display("[%0t] 2. One byte", $time);
        txn('{8'hA5}, '{8'h3C});

        $display("[%0t] 3. Protocol frame A2 00 00 00", $time);
        txn('{8'hA2, 8'h00, 8'h00, 8'h00}, '{8'hC5, 8'h12, 8'h34, 8'hF7});

        $display("[%0t] 4. Corner byte values", $time);
        txn('{8'h00, 8'hFF, 8'h80, 8'h01}, '{8'hFF, 8'h00, 8'h01, 8'h80});
        txn('{8'h55, 8'hAA}, '{8'hAA, 8'h55});

        $display("[%0t] 5. Back-to-back transactions", $time);
        repeat (10) txn('{$urandom, $urandom}, '{$urandom, $urandom});

        $display("[%0t] 6. Stall inside a transaction", $time);
        txn('{8'h11, 8'h22, 8'h33}, '{8'h44, 8'h55, 8'h66}, 0, 1, 137);
        txn('{8'h11, 8'h22, 8'h33}, '{8'h44, 8'h55, 8'h66}, 0, 2, 3);

        $display("[%0t] 7. 300 random transactions", $time);
        for (int k = 0; k < 300; k++) begin
            automatic int n = $urandom_range(1, 6);
            bytes_t d, r;
            d.delete();
            r.delete();
            repeat (n) begin
                d.push_back($urandom);
                r.push_back($urandom);
            end
            txn(d, r, $urandom_range(0, 40));
            if (k % 50 == 0) $display("[%0t]    txn %0d", $time, k);
        end

        $display("------------------------------------------------------------");
        $display("Transactions: %0d, ERRORS: %0d", n_txn, errors);
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
