// =============================================================================
// Тестбенч для i2c_master (i2c_txn + i2c_bit), 50 МГц, SCL 100 кГц
//
// На шине два ведомых из i2c_slave_model: «датчик» по адресу 0x48 (256 байт,
// пишется указатель, чтение с автоинкрементом) и «EEPROM» 0x50..0x53 (1024 байта,
// блоки по 256, страница записи 16 байт). Монитор линий (i2c_line_mon.svh)
// следит за временами Standard-mode и считает START / STOP на проводах.
// Классы:
//   Txn        - команда: адрес, чтение / запись, число байт, STOP, байты записи;
//   Result     - итог: res_len, флаги, принятые байты, события ev_*, rd_last;
//   RefModel   - эталон: память ведомых, указатели; предсказывает итог по Txn;
//   Scoreboard - сравнение итога с эталоном, счётчик ошибок.
// Проверки:
//   1. После сброса: линии свободны (1), cmd_ready = 1
//   2. Запись указателя (1 байт) и записи по 3 байта: память ведомого совпадает с эталоном
//   3. Запись указателя без STOP + чтение 2 байт (повторный START): на проводах 2 START и 1 STOP
//   4. Чтение 1, 5, 16 байт; cmd_len = 0 при чтении означает 1; rd_last только на последнем
//   5. Проверка адреса (запись 0 байт): ACK у 0x48, 0x50..0x53; NACK у 0x49, 0x2A; после NACK - STOP
//   6. NACK на байт данных (wr_limit): res_nack_data, res_len, STOP выдан
//   7. Цепочка без STOP: запись, запись, чтение; последняя транзакция со STOP
//   8. EEPROM: запись через границу страницы (20 байт с адреса 0xF8 блока 2) и чтение назад
//   9. Задержки потоков: wr_valid опаздывает до 300 тактов, rd_ready - до 300 тактов
//  10. Растяжка SCL ведомым 20 мкс на каждом такте
//  11. Растяжка дольше тайм-аута: res_timeout, линии отпущены, STOP не выдан; затем всё работает
//  12. SDA залипла в 0: res_arb; после отпускания всё работает
//  13. 120 случайных транзакций к 0x48, 0x50..0x53 и отсутствующим адресам
//  14. В конце: память обоих ведомых совпадает с эталоном
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps
`include "i2c_line_mon.svh"

module i2c_master_tb;

    localparam int  CLK_FREQ   = 50_000_000;
    localparam real CLK_NS     = 1.0e9 / CLK_FREQ;
    localparam int  TIMEOUT_US = 1_000;
    localparam int  LEN_W      = 8;

    typedef logic [7:0] bytes_t[$];

    class Txn;
        logic [6:0] addr;
        bit         read;
        int         len;        // число байт данных
        bit         stop;
        bytes_t     data;       // байты записи

        function new(logic [6:0] a, bit rd, int n, bit st);
            addr = a;  read = rd;  len = n;  stop = st;
        endfunction
    endclass

    class Result;
        int     len = 0;
        bit     nack_addr = 0, nack_data = 0, timeout = 0, arb = 0;
        bytes_t rd;
        int     n_start = 0, n_stop = 0, n_ack = 0, n_nack = 0;
        bit     last_ok = 1;    // rd_last был ровно на последнем байте

        function string str();
            return $sformatf("len=%0d nack_addr=%b nack_data=%b timeout=%b arb=%b rd=%p ev(start %0d stop %0d ack %0d nack %0d)",
                             len, nack_addr, nack_data, timeout, arb, rd, n_start, n_stop, n_ack, n_nack);
        endfunction
    endclass

    // эталон: «датчик» 0x48 (256 байт) и «EEPROM» 0x50..0x53 (1024 байта, страница 16)
    class RefModel;
        logic [7:0] m48 [256];
        logic [7:0] m50 [1024];
        int         p48 = 0, p50 = 0;

        function new();
            foreach (m48[i]) m48[i] = '0;
            foreach (m50[i]) m50[i] = '0;
        endfunction

        function bit present(logic [6:0] a);
            return (a == 7'h48) || (a[6:2] == 5'b10100);
        endfunction

        function Result predict(Txn t);
            Result r = new();
            r.n_start = 1;
            if (!present(t.addr)) begin
                r.nack_addr = 1;
                r.n_nack    = 1;
                r.n_stop    = 1;
                return r;
            end
            r.n_ack = 1;
            r.n_stop = t.stop ? 1 : 0;
            if (t.addr == 7'h48) begin
                if (!t.read) begin
                    for (int i = 0; i < t.len; i++)
                        if (i == 0) p48 = t.data[0];
                        else begin m48[p48] = t.data[i]; p48 = (p48 + 1) % 256; end
                    r.len   = t.len;
                    r.n_ack += t.len;
                end else begin
                    int n = (t.len == 0) ? 1 : t.len;
                    for (int i = 0; i < n; i++) begin r.rd.push_back(m48[p48]); p48 = (p48 + 1) % 256; end
                    r.len = n;
                end
            end else begin
                int blk = t.addr[1:0];
                if (!t.read) begin
                    for (int i = 0; i < t.len; i++)
                        if (i == 0) p50 = blk * 256 + t.data[0];
                        else begin m50[p50] = t.data[i]; p50 = (p50 / 16) * 16 + ((p50 + 1) % 16); end
                    r.len   = t.len;
                    r.n_ack += t.len;
                end else begin
                    int n = (t.len == 0) ? 1 : t.len;
                    p50 = blk * 256 + (p50 % 256);
                    for (int i = 0; i < n; i++) begin r.rd.push_back(m50[p50]); p50 = (p50 + 1) % 1024; end
                    r.len = n;
                end
            end
            return r;
        endfunction
    endclass

    class Scoreboard;
        int errors = 0;

        function void err(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction

        function void compare(string name, Result e, Result g);
            if (g.len !== e.len || g.nack_addr !== e.nack_addr || g.nack_data !== e.nack_data
                || g.timeout !== e.timeout || g.arb !== e.arb)
                err($sformatf("%s: result {%s}, expected {%s}", name, g.str(), e.str()));
            else if (g.rd.size() != e.rd.size())
                err($sformatf("%s: %0d bytes read, expected %0d", name, g.rd.size(), e.rd.size()));
            else begin
                for (int i = 0; i < e.rd.size(); i++)
                    if (g.rd[i] !== e.rd[i])
                        err($sformatf("%s: read byte %0d = %h, expected %h", name, i, g.rd[i], e.rd[i]));
            end
            if (g.n_start != e.n_start || g.n_stop != e.n_stop || g.n_ack != e.n_ack || g.n_nack != e.n_nack)
                err($sformatf("%s: events start %0d stop %0d ack %0d nack %0d, expected %0d %0d %0d %0d", name,
                              g.n_start, g.n_stop, g.n_ack, g.n_nack, e.n_start, e.n_stop, e.n_ack, e.n_nack));
            if (!g.last_ok) err($sformatf("%s: rd_last was not exactly on the last byte", name));
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

    always #(CLK_NS / 2) clk = ~clk;

    i2c_master #(
        .CLK_FREQ   (CLK_FREQ),
        .SCL_FREQ   (100_000),
        .TIMEOUT_US (TIMEOUT_US),
        .LEN_W      (LEN_W)
    ) dut (
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

    i2c_slave_model #(.ADDR (7'h48)) s48 (.scl (scl), .sda (sda));
    i2c_slave_model #(
        .ADDR       (7'h50),
        .ADDR_MASK  (7'h7C),
        .MEM_BITS   (10),
        .BLOCK_BITS (2),
        .PAGE_SIZE  (16)
    ) s50 (.scl (scl), .sda (sda));

    Scoreboard sb  = new();
    RefModel   ref_m = new();
    I2CLineMon mon = new();

    always @(posedge scl) if (rst_n) mon.scl_rise($realtime, sda);
    always @(negedge scl) if (rst_n) mon.scl_fall($realtime, sda);
    always @(posedge sda) if (rst_n) mon.sda_rise($realtime, scl);
    always @(negedge sda) if (rst_n) mon.sda_fall($realtime, scl);

    // -------------------------------------------------------------------------
    // Потоки: запись (источник) и чтение (приёмник) со случайными задержками
    // -------------------------------------------------------------------------
    bytes_t wq, rq;
    int     wr_gap_max = 0, rd_gap_max = 0;
    int     wr_wait = 0, rd_wait = 0;
    bit     wr_flush = 0;
    int     n_rd_bytes = 0, n_rd_last = 0;
    int     ev_n_start, ev_n_stop, ev_n_ack, ev_n_nack;
    bit     last_ok;

    always @(posedge clk) begin
        if (wr_flush) begin
            wr_valid <= 1'b0;
            wr_flush  = 0;
        end else if (wr_valid && wr_ready) begin
            void'(wq.pop_front());
            wr_valid <= 1'b0;
            wr_wait   = (wr_gap_max > 0) ? $urandom_range(0, wr_gap_max) : 0;
        end else if (!wr_valid && wq.size() > 0) begin
            if (wr_wait > 0) begin
                wr_wait--;
            end else begin
                wr_data  <= wq[0];
                wr_valid <= 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (rd_valid && rd_ready) begin
            rq.push_back(rd_data);
            n_rd_bytes++;
            if (rd_last) n_rd_last++;
            rd_wait = (rd_gap_max > 0) ? $urandom_range(0, rd_gap_max) : 0;
            if (rd_wait > 0) rd_ready <= 1'b0;
        end else if (!rd_ready) begin
            if (rd_wait > 0) rd_wait--;
            else             rd_ready <= 1'b1;
        end
    end

    // rd_valid и rd_data не меняются, пока rd_ready = 0
    logic       rd_valid_q = 1'b0, rd_ready_q = 1'b1;
    logic [7:0] rd_data_q;

    always @(posedge clk) begin
        if (rst_n && rd_valid_q && !rd_ready_q && (!rd_valid || rd_data !== rd_data_q))
            sb.err("rd_valid / rd_data changed before rd_ready");
        rd_valid_q <= rd_valid;
        rd_ready_q <= rd_ready;
        rd_data_q  <= rd_data;
    end

    always @(posedge clk) begin
        if (ev_start) ev_n_start++;
        if (ev_stop)  ev_n_stop++;
        if (ev_ack)   ev_n_ack++;
        if (ev_nack)  ev_n_nack++;
    end

    // -------------------------------------------------------------------------
    // Транзакция: команда, потоки, ожидание done, сбор итога
    // -------------------------------------------------------------------------
    task automatic run_txn(input Txn t, output Result r);
        r = new();
        wq = t.data;
        rq.delete();
        n_rd_bytes = 0;  n_rd_last = 0;
        ev_n_start = 0;  ev_n_stop = 0;  ev_n_ack = 0;  ev_n_nack = 0;
        @(posedge clk);
        cmd_addr  <= t.addr;
        cmd_read  <= t.read;
        cmd_len   <= LEN_W'(t.len);
        cmd_stop  <= t.stop;
        cmd_valid <= 1'b1;
        do @(posedge clk); while (!cmd_ready);
        cmd_valid <= 1'b0;
        do @(posedge clk); while (!done);
        r.len       = res_len;
        r.nack_addr = res_nack_addr;
        r.nack_data = res_nack_data;
        r.timeout   = res_timeout;
        r.arb       = res_arb;
        repeat (3) @(posedge clk);                          // события ev_* идут на такт раньше done
        r.rd        = rq;
        r.n_start   = ev_n_start;
        r.n_stop    = ev_n_stop;
        r.n_ack     = ev_n_ack;
        r.n_nack    = ev_n_nack;
        r.last_ok   = (n_rd_bytes == 0) || (n_rd_last == 1 && rq.size() == n_rd_bytes);
        wq.delete();
        wr_flush = 1;
        @(posedge clk);
    endtask

    function automatic Txn mk_wr(logic [6:0] a, bytes_t d, bit stop = 1);
        Txn t = new(a, 0, d.size(), stop);
        t.data = d;
        return t;
    endfunction

    function automatic Txn mk_rd(logic [6:0] a, int n, bit stop = 1);
        Txn t = new(a, 1, n, stop);
        return t;
    endfunction

    // транзакция по эталону: сравнить итог и состояние шины
    task automatic check_txn(input string name, input Txn t);
        Result e, g;
        int    starts0, stops0;
        starts0 = mon.n_start;
        stops0  = mon.n_stop;
        e = ref_m.predict(t);
        run_txn(t, g);
        sb.compare(name, e, g);
        if (mon.n_start - starts0 != 1) sb.err($sformatf("%s: %0d START on the wires, expected 1", name, mon.n_start - starts0));
        if (mon.n_stop - stops0 != e.n_stop)
            sb.err($sformatf("%s: %0d STOP on the wires, expected %0d", name, mon.n_stop - stops0, e.n_stop));
        if (e.n_stop == 1) begin
            if (scl !== 1'b1 || sda !== 1'b1) sb.err($sformatf("%s: bus is not idle after STOP SCL=%b SDA=%b", name, scl, sda));
        end else begin
            if (scl !== 1'b0) sb.err($sformatf("%s: SCL = %b, the bus must stay held without STOP", name, scl));
        end
    endtask

    function automatic bytes_t rnd_bytes(int n);
        bytes_t d;
        d.delete();
        repeat (n) d.push_back($urandom);
        return d;
    endfunction

    task automatic compare_memories(input string name);
        for (int i = 0; i < 256; i++)
            if (s48.mem[i] !== ref_m.m48[i]) sb.err($sformatf("%s: 0x48 mem[%0d] = %h, expected %h", name, i, s48.mem[i], ref_m.m48[i]));
        for (int i = 0; i < 1024; i++)
            if (s50.mem[i] !== ref_m.m50[i]) sb.err($sformatf("%s: 0x50 mem[%0d] = %h, expected %h", name, i, s50.mem[i], ref_m.m50[i]));
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        Txn    t;
        Result g;
        bytes_t d;
        $timeformat(-6, 2, " us", 0);
        for (int i = 0; i < 256; i++)  begin s48.mem[i] = $urandom; ref_m.m48[i] = s48.mem[i]; end
        for (int i = 0; i < 1024; i++) begin s50.mem[i] = $urandom; ref_m.m50[i] = s50.mem[i]; end
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        $display("[%0t] 1. Reset state", $time);
        if (scl !== 1'b1 || sda !== 1'b1 || cmd_ready !== 1'b1)
            sb.err($sformatf("after reset SCL=%b SDA=%b cmd_ready=%b", scl, sda, cmd_ready));

        $display("[%0t] 2. Writes", $time);
        d = '{8'h20};
        check_txn("pointer only", mk_wr(7'h48, d));
        d = '{8'h10, 8'hDE, 8'hAD};
        check_txn("write 3", mk_wr(7'h48, d));
        d = '{8'h00, 8'hFF};
        check_txn("write 2", mk_wr(7'h48, d));
        compare_memories("after writes");

        $display("[%0t] 3. Repeated START: pointer without STOP + read", $time);
        d = '{8'h10};
        check_txn("pointer, no STOP", mk_wr(7'h48, d, 0));
        check_txn("read after repeated START", mk_rd(7'h48, 2));
        if (s48.rx_log.size() == 0) sb.err("slave log is empty");

        $display("[%0t] 4. Reads", $time);
        d = '{8'h00};
        check_txn("pointer 0", mk_wr(7'h48, d));
        check_txn("read 1", mk_rd(7'h48, 1));
        check_txn("read 5", mk_rd(7'h48, 5));
        check_txn("read 16", mk_rd(7'h48, 16));
        check_txn("read len 0 (= 1)", mk_rd(7'h48, 0));
        if (s48.n_master_nack == 0) sb.err("slave saw no NACK from the master at the end of a read");

        $display("[%0t] 5. Address probes", $time);
        begin
            logic [6:0] probe [6];
            probe = '{7'h48, 7'h50, 7'h51, 7'h52, 7'h53, 7'h49};
            foreach (probe[i]) begin
                d.delete();
                check_txn($sformatf("probe 0x%h", probe[i]), mk_wr(probe[i], d));
            end
            d.delete();
            check_txn("probe 0x2A", mk_wr(7'h2A, d));
            check_txn("read of an absent device", mk_rd(7'h2A, 3));
            check_txn("probe 0x48 after NACKs", mk_wr(7'h48, d));
        end

        $display("[%0t] 6. NACK on a data byte", $time);
        s48.wr_limit = 1;                                   // указатель принимается, дальше NACK
        d = '{8'h30, 8'h11, 8'h22};
        t = mk_wr(7'h48, d, 0);                             // без STOP: после NACK STOP всё равно
        begin
            automatic int st0 = mon.n_stop;
            run_txn(t, g);
            if (!g.nack_data || g.nack_addr || g.len != 1 || g.n_stop != 1 || g.n_nack != 1 || g.n_ack != 2)
                sb.err($sformatf("NACK on data: {%s}", g.str()));
            if (mon.n_stop - st0 != 1) sb.err("NACK on data: STOP was not issued");
            if (scl !== 1'b1 || sda !== 1'b1) sb.err("NACK on data: bus is not idle");
        end
        s48.wr_limit = 1_000_000;
        ref_m.p48 = 8'h30;                                  // указатель записан, данные нет
        check_txn("write after NACK", mk_wr(7'h48, '{8'h31, 8'h77}));

        $display("[%0t] 7. Chain without STOP", $time);
        check_txn("chain 1: pointer", mk_wr(7'h48, '{8'h40}, 0));
        check_txn("chain 2: write", mk_wr(7'h48, '{8'h40, 8'hA1, 8'hA2}, 0));
        check_txn("chain 3: pointer", mk_wr(7'h48, '{8'h40}, 0));
        check_txn("chain 4: read", mk_rd(7'h48, 3));
        compare_memories("after chain");

        $display("[%0t] 8. EEPROM page boundary", $time);
        d = rnd_bytes(21);
        d[0] = 8'hF8;                                       // с адреса 0xF8 блока 2 на 20 байт - через границу страницы
        check_txn("EEPROM write 20", mk_wr(7'h52, d));
        check_txn("EEPROM pointer", mk_wr(7'h52, '{8'hF0}, 0));
        check_txn("EEPROM read 32", mk_rd(7'h52, 32));
        check_txn("EEPROM read other block", mk_rd(7'h50, 8));
        compare_memories("after EEPROM");

        $display("[%0t] 9. Stream delays", $time);
        wr_gap_max = 300;
        rd_gap_max = 300;
        d = rnd_bytes(5);
        check_txn("write with delays", mk_wr(7'h48, d));
        check_txn("pointer with delays", mk_wr(7'h48, '{8'h00}, 0));
        check_txn("read with delays", mk_rd(7'h48, 6));
        wr_gap_max = 0;
        rd_gap_max = 0;

        $display("[%0t] 10. Clock stretching by the slave", $time);
        mon.check_period = 1'b0;
        s48.stretch_ns = 20_000;
        s50.stretch_ns = 20_000;
        check_txn("stretched write", mk_wr(7'h48, '{8'h08, 8'h5A, 8'hA5}));
        check_txn("stretched pointer", mk_wr(7'h48, '{8'h08}, 0));
        check_txn("stretched read", mk_rd(7'h48, 3));
        s48.stretch_ns = 0;
        s50.stretch_ns = 0;
        mon.check_period = 1'b1;

        $display("[%0t] 11. Stretching longer than the timeout", $time);
        mon.enable = 1'b0;
        s48.stretch_ns = 3_000_000;
        t = mk_wr(7'h48, '{8'h01, 8'h02});
        run_txn(t, g);
        if (!g.timeout || g.arb || g.nack_addr || g.nack_data || g.n_stop != 0)
            sb.err($sformatf("timeout: {%s}", g.str()));
        if (dut.u_bit.scl_low !== 1'b0 || dut.u_bit.sda_low !== 1'b0) sb.err("timeout: lines are not released");
        s48.stretch_ns = 0;
        #(4_000_000.0);
        if (scl !== 1'b1 || sda !== 1'b1) sb.err($sformatf("timeout: bus is not idle SCL=%b SDA=%b", scl, sda));
        mon.enable = 1'b1;
        // записанное до тайм-аута (если успело) эталон не знает: перечитаем память ведомого в эталон
        for (int i = 0; i < 256; i++) ref_m.m48[i] = s48.mem[i];
        ref_m.p48 = s48.ptr;
        check_txn("write after timeout", mk_wr(7'h48, '{8'h50, 8'h99}));
        check_txn("pointer after timeout", mk_wr(7'h48, '{8'h50}, 0));
        check_txn("read after timeout", mk_rd(7'h48, 1));

        $display("[%0t] 12. SDA stuck low", $time);
        mon.enable = 1'b0;
        s48.hold_sda = 1'b1;
        #(20_000.0);
        t = mk_wr(7'h48, '{8'h60});
        run_txn(t, g);
        if (!g.arb || g.timeout || g.n_stop != 0 || g.n_start != 0)
            sb.err($sformatf("SDA stuck: {%s}", g.str()));
        if (dut.u_bit.scl_low !== 1'b0 || dut.u_bit.sda_low !== 1'b0) sb.err("SDA stuck: lines are not released");
        s48.hold_sda = 1'b0;
        #(40_000.0);
        mon.enable = 1'b1;
        check_txn("write after stuck SDA", mk_wr(7'h48, '{8'h60, 8'h12}));
        check_txn("pointer after stuck SDA", mk_wr(7'h48, '{8'h60}, 0));
        check_txn("read after stuck SDA", mk_rd(7'h48, 1));

        $display("[%0t] 13. 120 random transactions", $time);
        for (int k = 0; k < 120; k++) begin
            automatic logic [6:0] addrs [8] = '{7'h48, 7'h48, 7'h50, 7'h51, 7'h52, 7'h53, 7'h49, 7'h2A};
            automatic logic [6:0] a = addrs[$urandom_range(0, 7)];
            automatic bit         rd = $urandom_range(0, 1);
            automatic bit         st = ($urandom_range(0, 3) != 0) || (k == 119);
            automatic int         n  = rd ? $urandom_range(0, 12) : $urandom_range(0, 20);
            automatic Txn         r;
            if (rd) r = mk_rd(a, n, st);
            else    r = mk_wr(a, rnd_bytes(n), st);
            wr_gap_max = ($urandom_range(0, 4) == 0) ? 40 : 0;
            rd_gap_max = ($urandom_range(0, 4) == 0) ? 40 : 0;
            check_txn($sformatf("rand%0d", k), r);
        end
        wr_gap_max = 0;
        rd_gap_max = 0;

        $display("[%0t] 14. Memories vs reference", $time);
        compare_memories("final");

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d (testbench) + %0d (bus monitor)", sb.errors, mon.errors);
        if (sb.errors == 0 && mon.errors == 0) $display("TEST PASSED");
        else                                   $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #200ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
