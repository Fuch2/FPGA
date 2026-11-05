// =============================================================================
// Тестбенч для dt_model (i2c_slave + temp_gen + регистры LM75A), 50 МГц, SCL 100 кГц
//
// Модель разговаривает с настоящим i2c_master из этапа 4. Период обновления
// температуры в тестбенче - 1 мс (UPDATE_MS = 1), чтобы последовательность успевала
// меняться. Монитор линий I2CLineMon следит за временами Standard-mode.
// Классы:
//   TempHist   - история значений температуры внутри модели (время, значение);
//                проверяет, что прочитанное по шине значение было текущим в течение
//                транзакции (значит, байты не «рваные»: старший от одного, младший
//                от другого значения)
//   Scoreboard - счётчик ошибок
// Проверки:
//   1. После сброса линии свободны
//   2. Значения регистров после включения: Config 0x00, Thyst 0x4B00, Tos 0x5000
//   3. Режим «константа»: 25.375 °C = 0x1960, -1.0 °C = 0xFF00, -55 °C, +125 °C;
//      младшие 5 бит равны 0
//   4. Указатель сохраняется между транзакциями; чтение больше двух байт: старший,
//      младший, старший, ... одного значения; Config читается одним и тем же байтом
//   5. Запись Thyst и Tos (младший байт маскируется до D7), запись в регистр
//      температуры и запись лишних байт в Config ничего не портят
//   6. Конфигурация shutdown: температура перестаёт обновляться; после 0x00 снова
//      обновляется
//   7. Чужой адрес и present = 0: NACK на адрес; после present = 1 ответ есть
//   8. Рваное чтение: пила с большим шагом, 150 чтений в случайные моменты, каждое
//      значение было в истории за время транзакции
//   9. Линейный режим (рост до gen_hi и остановка), пила (возврат в gen_lo),
//      шум (значения в [lo, lo + mask])
//  10. Монитор линий: ни одного нарушения времён
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps
`include "i2c_line_mon.svh"

module dt_model_tb;

    localparam int  CLK_FREQ  = 50_000_000;
    localparam real CLK_NS    = 1.0e9 / CLK_FREQ;
    localparam int  LEN_W     = 8;
    localparam logic [6:0] DT_ADDR = 7'h48;

    typedef logic [7:0] bytes_t[$];

    class TempHist;
        realtime     t [$];
        logic [15:0] v [$];

        function void add(realtime tm, logic [15:0] val);
            t.push_back(tm);
            v.push_back(val);
        endfunction

        // значение val было текущим в какой-то момент отрезка [t0, t1]
        function bit seen(realtime t0, realtime t1, logic [15:0] val);
            for (int i = 0; i < t.size(); i++) begin
                realtime t_end;
                t_end = (i + 1 < t.size()) ? t[i + 1] : 1.0e18;
                if (t[i] <= t1 && t_end >= t0 && v[i] === val) return 1'b1;
            end
            return 1'b0;
        endfunction
    endclass

    class Scoreboard;
        int errors = 0;

        function void err(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic             clk = 1'b0, rst_n = 1'b0;
    logic             cmd_valid = 1'b0, cmd_read = 1'b0, cmd_stop = 1'b0;
    logic [6:0]       cmd_addr = '0;
    logic [LEN_W-1:0] cmd_len = '0;
    logic [7:0]       wr_data = '0;
    logic             wr_valid = 1'b0;
    logic             rd_ready = 1'b1;
    logic             cmd_ready, wr_ready, rd_valid, rd_last, done;
    logic [7:0]       rd_data;
    logic [LEN_W-1:0] res_len;
    logic             res_nack_addr, res_nack_data, res_timeout, res_arb;
    logic             ev_start, ev_stop, ev_ack, ev_nack;
    tri1              scl, sda;

    logic               present = 1'b1, gen_restart = 1'b0;
    logic        [1:0]  gen_mode = '0;
    logic signed [10:0] gen_lo = 11'sd203, gen_hi = 11'sd1000;
    logic        [10:0] gen_step = 11'd8;
    logic        [7:0]  gen_noise_mask = 8'h0F;
    logic               sda_low;

    always #(CLK_NS / 2) clk = ~clk;

    i2c_master #(
        .CLK_FREQ (CLK_FREQ),
        .SCL_FREQ (100_000),
        .LEN_W    (LEN_W)
    ) master (
        .clk (clk), .rst_n (rst_n),
        .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_addr (cmd_addr), .cmd_read (cmd_read),
        .cmd_len (cmd_len), .cmd_stop (cmd_stop),
        .wr_data (wr_data), .wr_valid (wr_valid), .wr_ready (wr_ready),
        .rd_data (rd_data), .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_last (rd_last),
        .done (done), .res_len (res_len), .res_nack_addr (res_nack_addr), .res_nack_data (res_nack_data),
        .res_timeout (res_timeout), .res_arb (res_arb),
        .ev_start (ev_start), .ev_stop (ev_stop), .ev_ack (ev_ack), .ev_nack (ev_nack),
        .i2c_scl (scl), .i2c_sda (sda)
    );

    dt_model #(
        .CLK_FREQ  (CLK_FREQ),
        .ADDR      (DT_ADDR),
        .UPDATE_MS (1)
    ) dut (
        .clk (clk), .rst_n (rst_n),
        .present (present), .gen_restart (gen_restart), .gen_mode (gen_mode),
        .gen_lo (gen_lo), .gen_hi (gen_hi), .gen_step (gen_step), .gen_noise_mask (gen_noise_mask),
        .scl_in (scl), .sda_in (sda), .sda_low (sda_low)
    );

    assign sda = sda_low ? 1'b0 : 1'bz;           // открытый сток ведомого

    Scoreboard sb   = new();
    TempHist   hist = new();
    I2CLineMon mon  = new();

    always @(posedge scl) if (rst_n) mon.scl_rise($realtime, sda);
    always @(negedge scl) if (rst_n) mon.scl_fall($realtime, sda);
    always @(posedge sda) if (rst_n) mon.sda_rise($realtime, scl);
    always @(negedge sda) if (rst_n) mon.sda_fall($realtime, scl);

    always @(dut.temp_word) hist.add($realtime, dut.temp_word);

    // -------------------------------------------------------------------------
    // Потоки записи и чтения
    // -------------------------------------------------------------------------
    bytes_t wq, rq;
    bit     wr_flush = 0;

    always @(posedge clk) begin
        if (wr_flush) begin
            wr_valid <= 1'b0;
            wr_flush  = 0;
        end else if (wr_valid && wr_ready) begin
            void'(wq.pop_front());
            wr_valid <= 1'b0;
        end else if (!wr_valid && wq.size() > 0) begin
            wr_data  <= wq[0];
            wr_valid <= 1'b1;
        end
    end

    always @(posedge clk) if (rd_valid && rd_ready) rq.push_back(rd_data);

    // -------------------------------------------------------------------------
    // Транзакция: итог в переменных
    // -------------------------------------------------------------------------
    bit     r_nack_addr, r_nack_data, r_err;
    int     r_len;
    bytes_t r_rd;

    task automatic txn(input logic [6:0] a, input bit rd, input int n, input bit stop, input bytes_t wdata);
        wq = wdata;
        rq.delete();
        @(posedge clk);
        cmd_addr  <= a;
        cmd_read  <= rd;
        cmd_len   <= LEN_W'(n);
        cmd_stop  <= stop;
        cmd_valid <= 1'b1;
        do @(posedge clk); while (!cmd_ready);
        cmd_valid <= 1'b0;
        do @(posedge clk); while (!done);
        r_len       = res_len;
        r_nack_addr = res_nack_addr;
        r_nack_data = res_nack_data;
        r_err       = res_timeout || res_arb;
        repeat (3) @(posedge clk);
        r_rd = rq;
        wq.delete();
        wr_flush = 1;
        @(posedge clk);
    endtask

    // запись байтов регистра: первый - указатель
    task automatic wr_regs(input bytes_t d);
        txn(DT_ADDR, 1'b0, d.size(), 1'b1, d);
        if (r_nack_addr || r_nack_data || r_err || r_len != d.size())
            sb.err($sformatf("write %p failed: len %0d nack_addr %b nack_data %b", d, r_len, r_nack_addr, r_nack_data));
    endtask

    // чтение n байт с текущего указателя
    task automatic rd_cur(input int n, output bytes_t d);
        bytes_t none;
        none.delete();
        txn(DT_ADDR, 1'b1, n, 1'b1, none);
        if (r_nack_addr || r_nack_data || r_err || r_rd.size() != n)
            sb.err($sformatf("read %0d bytes failed: got %0d, nack_addr %b", n, r_rd.size(), r_nack_addr));
        d = r_rd;
    endtask

    // указатель + чтение n байт через повторный START; t0, t1 - границы по времени
    task automatic rd_reg(input logic [7:0] ptr, input int n, output bytes_t d,
                          output realtime t0, output realtime t1);
        bytes_t w;
        w.delete();
        w.push_back(ptr);
        t0 = $realtime;
        txn(DT_ADDR, 1'b0, 1, 1'b0, w);
        if (r_nack_addr || r_nack_data || r_err) sb.err($sformatf("pointer %0d write failed", ptr));
        rd_cur(n, d);
        t1 = $realtime;
    endtask

    function automatic logic [15:0] word(bytes_t d, int i = 0);
        return {d[i], d[i + 1]};
    endfunction

    // выбрать режим и параметры; последовательность начинается заново
    task automatic set_gen(input int mode, input int lo, input int hi, input int step, input int mask);
        @(negedge clk);
        gen_mode = mode;  gen_lo = lo;  gen_hi = hi;  gen_step = step;  gen_noise_mask = mask;
        @(negedge clk);
        gen_restart = 1'b1;
        @(negedge clk);
        gen_restart = 1'b0;
    endtask

    task automatic wait_ms(input int ms);
        #(ms * 1_000_000.0);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        bytes_t d, d2, w;
        realtime t0, t1;
        logic [15:0] x, prev;
        int n_bad, n_diff;
        $timeformat(-6, 2, " us", 0);
        hist.add(0, '0);
        #(CLK_NS * 5);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // 1. линии свободны
        if (scl !== 1'b1 || sda !== 1'b1) sb.err("bus is not idle after reset");

        // 2. регистры после включения
        rd_reg(8'h01, 1, d, t0, t1);
        if (d[0] !== 8'h00) sb.err($sformatf("Config = %h, expected 00", d[0]));
        rd_reg(8'h02, 2, d, t0, t1);
        if (word(d) !== 16'h4B00) sb.err($sformatf("Thyst = %h, expected 4B00", word(d)));
        rd_reg(8'h03, 2, d, t0, t1);
        if (word(d) !== 16'h5000) sb.err($sformatf("Tos = %h, expected 5000", word(d)));

        // 3. константа
        set_gen(0, 203, 1000, 8, 0);                    // 25.375 °C
        wait_ms(2);
        rd_reg(8'h00, 2, d, t0, t1);
        if (word(d) !== 16'h1960) sb.err($sformatf("25.375 degC = %h, expected 1960", word(d)));
        set_gen(0, -8, 1000, 8, 0);                     // -1.0 °C
        wait_ms(2);
        rd_reg(8'h00, 2, d, t0, t1);
        if (word(d) !== 16'hFF00) sb.err($sformatf("-1.0 degC = %h, expected FF00", word(d)));
        set_gen(0, -440, 1000, 8, 0);                   // -55 °C
        wait_ms(2);
        rd_reg(8'h00, 2, d, t0, t1);
        if (word(d) !== {11'(-440), 5'b0}) sb.err($sformatf("-55 degC = %h, expected %h", word(d), {11'(-440), 5'b0}));
        set_gen(0, 1000, 1000, 8, 0);                   // +125 °C
        wait_ms(2);
        rd_reg(8'h00, 2, d, t0, t1);
        if (word(d) !== 16'h7D00) sb.err($sformatf("125 degC = %h, expected 7D00", word(d)));
        if (d[1][4:0] !== 5'b0) sb.err("low 5 bits of temperature are not zero");

        // 4. указатель сохраняется, длинное чтение, Config
        set_gen(0, 203, 1000, 8, 0);
        wait_ms(2);
        rd_reg(8'h03, 2, d, t0, t1);
        rd_cur(2, d);
        if (word(d) !== 16'h5000) sb.err($sformatf("pointer not kept: read %h, expected 5000", word(d)));
        rd_reg(8'h00, 6, d, t0, t1);
        if (word(d, 0) !== 16'h1960 || word(d, 2) !== 16'h1960 || word(d, 4) !== 16'h1960)
            sb.err($sformatf("6-byte read of temperature: %p", d));
        rd_reg(8'h01, 3, d, t0, t1);
        if (d[0] !== 8'h00 || d[1] !== 8'h00 || d[2] !== 8'h00) sb.err($sformatf("3-byte Config read: %p", d));

        // 5. запись Thyst, Tos; запись в регистр температуры
        w.delete();  w = '{8'h02, 8'h19, 8'h60};
        wr_regs(w);
        rd_cur(2, d);                                   // указатель остался на Thyst
        if (word(d) !== 16'h1900) sb.err($sformatf("Thyst after write = %h, expected 1900", word(d)));
        w.delete();  w = '{8'h03, 8'h2A, 8'h80};
        wr_regs(w);
        rd_cur(2, d);
        if (word(d) !== 16'h2A80) sb.err($sformatf("Tos after write = %h, expected 2A80", word(d)));
        w.delete();  w = '{8'h00, 8'h12, 8'h34};        // температура - только чтение
        wr_regs(w);
        rd_cur(2, d);
        if (word(d) !== 16'h1960) sb.err($sformatf("temperature register changed by write: %h", word(d)));
        rd_reg(8'h02, 2, d, t0, t1);
        if (word(d) !== 16'h1900) sb.err($sformatf("Thyst changed: %h", word(d)));

        // 6. shutdown: температура останавливается
        set_gen(2, 0, 1000, 8, 0);                      // пила: меняется каждый миллисекунд
        wait_ms(2);
        rd_reg(8'h00, 2, d, t0, t1);
        wait_ms(3);
        rd_reg(8'h00, 2, d2, t0, t1);
        if (word(d) === word(d2)) sb.err("temperature does not change in saw mode");
        w.delete();  w = '{8'h01, 8'h01};
        wr_regs(w);                                     // Config = shutdown
        rd_reg(8'h01, 1, d, t0, t1);
        if (d[0] !== 8'h01) sb.err($sformatf("Config = %h, expected 01", d[0]));
        rd_reg(8'h00, 2, d, t0, t1);
        wait_ms(5);
        rd_reg(8'h00, 2, d2, t0, t1);
        if (word(d) !== word(d2)) sb.err($sformatf("temperature changed in shutdown: %h -> %h", word(d), word(d2)));
        w.delete();  w = '{8'h01, 8'h00};
        wr_regs(w);
        rd_reg(8'h00, 2, d, t0, t1);
        wait_ms(5);
        rd_reg(8'h00, 2, d2, t0, t1);
        if (word(d) === word(d2)) sb.err("temperature does not resume after shutdown is cleared");

        // 7. чужой адрес и present = 0
        w.delete();
        txn(7'h49, 1'b1, 1, 1'b1, w);
        if (!r_nack_addr) sb.err("0x49 must NACK");
        present = 1'b0;
        txn(DT_ADDR, 1'b1, 1, 1'b1, w);
        if (!r_nack_addr) sb.err("present = 0: expected NACK");
        txn(DT_ADDR, 1'b0, 0, 1'b1, w);
        if (!r_nack_addr) sb.err("present = 0: address probe expected NACK");
        present = 1'b1;
        txn(DT_ADDR, 1'b0, 0, 1'b1, w);
        if (r_nack_addr) sb.err("present = 1: expected ACK");
        rd_reg(8'h00, 2, d, t0, t1);                    // после NACK-ов связь работает
        if (!hist.seen(t0, t1, word(d))) sb.err("read after NACKs: wrong value");

        // 8. рваное чтение: пила с большим шагом, чтения в случайные моменты
        set_gen(2, -1000, 1000, 311, 0);
        n_bad = 0;
        for (int i = 0; i < 150; i++) begin
            #($urandom_range(0, 1_500_000));
            rd_reg(8'h00, 2, d, t0, t1);
            if (!hist.seen(t0, t1, word(d))) begin
                n_bad++;
                sb.err($sformatf("read %0d: value %h was not current during the transaction", i, word(d)));
            end
            if (d[1][4:0] !== 5'b0) sb.err("low 5 bits not zero");
        end

        // 9. режимы
        set_gen(1, 0, 40, 8, 0);                        // линейный рост 0 .. 5 °C
        prev = '0;
        for (int i = 0; i < 12; i++) begin
            wait_ms(1);
            rd_reg(8'h00, 2, d, t0, t1);
            x = word(d);
            if (x < prev) sb.err($sformatf("linear: value went down %h -> %h", prev, x));
            prev = x;
        end
        if (x !== {11'sd40, 5'b0}) sb.err($sformatf("linear: final %h, expected %h", x, {11'sd40, 5'b0}));

        set_gen(2, 0, 40, 16, 0);                       // пила: 0, 16, 32, затем снова 0
        n_diff = 0;
        for (int i = 0; i < 16; i++) begin
            wait_ms(1);
            rd_reg(8'h00, 2, d, t0, t1);
            x = word(d);
            if (x[4:0] !== 5'b0 || ($signed(x[15:5]) % 16 != 0) || $signed(x[15:5]) < 0 || $signed(x[15:5]) > 40)
                sb.err($sformatf("saw: bad value %h", x));
            if (x === 16'h0000) n_diff++;
        end
        if (n_diff == 0) sb.err("saw: never returned to gen_lo");

        set_gen(3, 100, 300, 1, 8'h07);                 // шум: 100 .. 107
        n_diff = 0;
        for (int i = 0; i < 20; i++) begin
            wait_ms(1);
            rd_reg(8'h00, 2, d, t0, t1);
            x = word(d);
            if ($signed(x[15:5]) < 100 || $signed(x[15:5]) > 107)
                sb.err($sformatf("noise: value %0d out of 100..107", $signed(x[15:5])));
            if (x !== prev) n_diff++;
            prev = x;
        end
        if (n_diff < 5) sb.err($sformatf("noise: value changed only %0d times", n_diff));

        // 10. монитор линий
        if (mon.errors != 0) sb.err($sformatf("line monitor: %0d violations", mon.errors));
        if (mon.n_start < 100) sb.err($sformatf("line monitor saw only %0d STARTs", mon.n_start));

        repeat (10) @(posedge clk);
        if (sb.errors == 0) $display("TEST PASSED");
        else                $display("TEST FAILED: %0d errors", sb.errors);
        $finish;
    end

endmodule
