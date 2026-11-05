// =============================================================================
// Тестбенч для eeprom_model: имитатор AT24C08, проверяется через настоящий i2c_master
//
// 50 МГц, SCL 100 кГц, цикл записи имитатора 5000 мкс (как у AT24C08). Эталон - класс
// RefMem (память 1024 байта, указатель, буфер страницы 16 байт, адрес блока в адресе
// устройства), написанный независимо от модели.
// Проверки:
//   1. Новая микросхема: вся память 0xFF, чтение по всем четырём блокам
//   2. Запись байта 0x5A по адресу 0x123 (блок 1): ACK; сразу после STOP адрес не
//      подтверждается (NACK) 5000 мкс +-1 такт (по сигналу busy), опрос ACK видит
//      десятки NACK и заканчивается ACK; запись и чтение другого адреса в это время
//      тоже получают NACK и ничего не меняют; wr_cycles = 1
//   3. Страница: запись 5 байт в конец страницы (адрес переворачивается внутри
//      страницы), 16 и 20 байт (затирание начала буфера последними байтами); один
//      цикл записи на транзакцию; содержимое памяти совпадает с эталоном
//   4. Чтение: произвольное (адрес + повторный START), последовательное через границу
//      блока 256 байт и по концу памяти (1023 -> 0), чтение с текущего адреса после
//      записи и после чтения
//   5. Запись одного адреса без данных и запись, прерванная повторным START вместо STOP,
//      цикла записи не запускают и память не меняют
//   6. Адреса устройства 0x50..0x53 отвечают (блок P1 P0 - старшие биты адреса ячейки),
//      0x4F и 0x54 - NACK; present = 0 - NACK на все адреса
//   7. Случайные записи (1..20 байт) и чтения (1..40 байт) по случайным адресам
//      сравниваются с эталоном; в конце вся память читается за одну транзакцию
//      (1024 байта) и сравнивается с эталоном и с содержимым модели
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module eeprom_model_tb;

    localparam int  CLK_FREQ  = 50_000_000;
    localparam real CLK_NS    = 1.0e9 / CLK_FREQ;
    localparam int  WRITE_US  = 5_000;
    localparam int  LEN_W     = 11;                 // для чтения всей памяти одной транзакцией

    typedef logic [7:0] bytes_t[$];

    class Res;
        bit     nack_addr = 0, nack_data = 0, timeout = 0, arb = 0;
        int     len = 0;
        bytes_t rd;
    endclass

    // эталон AT24C08: память, текущий адрес, буфер страницы
    class RefMem;
        logic [7:0] m [1024];
        int         ptr = 0;

        function new();
            foreach (m[i]) m[i] = 8'hFF;
        endfunction

        // адрес устройства: номер блока - старшие биты адреса ячейки
        function void addr_phase(int blk);
            ptr = blk * 256 + (ptr % 256);
        endfunction

        // запись: d[0] - младший байт адреса, остальные - данные; commit = 0 - запись
        // прервана (нет STOP). Возвращает число байтов данных
        function int write(int blk, bytes_t d, bit commit = 1);
            logic [7:0] pb [16];
            bit         pv [16];
            int         base;
            addr_phase(blk);
            if (d.size() == 0) return 0;
            ptr  = blk * 256 + d[0];
            base = (ptr / 16) * 16;
            foreach (pv[i]) pv[i] = 0;
            for (int i = 1; i < d.size(); i++) begin
                pb[ptr % 16] = d[i];
                pv[ptr % 16] = 1;
                ptr = base + ((ptr + 1) % 16);
            end
            if (commit)
                for (int k = 0; k < 16; k++)
                    if (pv[k]) m[base + k] = pb[k];
            return d.size() - 1;
        endfunction

        function bytes_t read(int blk, int n);
            bytes_t r;
            addr_phase(blk);
            for (int i = 0; i < n; i++) begin
                r.push_back(m[ptr]);
                ptr = (ptr + 1) % 1024;
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
    tri1              scl, sda;
    logic             present = 1'b1;
    logic             model_busy, model_sda_low;
    logic [15:0]      wr_cycles;

    always #(CLK_NS / 2) clk = ~clk;

    i2c_master #(
        .CLK_FREQ   (CLK_FREQ),
        .SCL_FREQ   (100_000),
        .TIMEOUT_US (1_000),
        .LEN_W      (LEN_W)
    ) master (
        .clk (clk), .rst_n (rst_n),
        .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_addr (cmd_addr), .cmd_read (cmd_read),
        .cmd_len (cmd_len), .cmd_stop (cmd_stop),
        .wr_data (wr_data), .wr_valid (wr_valid), .wr_ready (wr_ready),
        .rd_data (rd_data), .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_last (rd_last),
        .done (done), .res_len (res_len), .res_nack_addr (res_nack_addr), .res_nack_data (res_nack_data),
        .res_timeout (res_timeout), .res_arb (res_arb),
        .ev_start (), .ev_stop (), .ev_ack (), .ev_nack (),
        .i2c_scl (scl), .i2c_sda (sda)
    );

    eeprom_model #(
        .CLK_FREQ   (CLK_FREQ),
        .ADDR       (7'h50),
        .MEM_BYTES  (1024),
        .PAGE_BYTES (16),
        .WRITE_US   (WRITE_US)
    ) model (
        .clk (clk), .rst_n (rst_n), .present (present), .busy (model_busy), .wr_cycles (wr_cycles),
        .scl_in (scl), .sda_in (sda), .sda_low (model_sda_low)
    );
    assign sda = model_sda_low ? 1'b0 : 1'bz;

    Scoreboard sb    = new();
    RefMem     ref_m = new();

    // длительность цикла записи по сигналу busy
    realtime t_rise = 0, t_fall = 0;
    int      n_busy_rises = 0;

    always @(posedge model_busy) begin t_rise = $realtime; n_busy_rises++; end
    always @(negedge model_busy) t_fall = $realtime;

    // -------------------------------------------------------------------------
    // Потоки: запись (источник) и чтение (приёмник)
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

    always @(posedge clk) begin
        if (rd_valid && rd_ready) rq.push_back(rd_data);
    end

    // -------------------------------------------------------------------------
    // Транзакция I2C
    // -------------------------------------------------------------------------
    task automatic txn(input logic [6:0] a, input bit rd, input int len, input bit stop,
                       input bytes_t wd, output Res r);
        r = new();
        wq = wd;
        rq.delete();
        @(posedge clk);
        cmd_addr  <= a;
        cmd_read  <= rd;
        cmd_len   <= LEN_W'(len);
        cmd_stop  <= stop;
        cmd_valid <= 1'b1;
        do @(posedge clk); while (!cmd_ready);
        cmd_valid <= 1'b0;
        do @(posedge clk); while (!done);
        r.len       = res_len;
        r.nack_addr = res_nack_addr;
        r.nack_data = res_nack_data;
        r.timeout   = res_timeout;
        r.arb       = res_arb;
        repeat (3) @(posedge clk);
        r.rd = rq;
        wq.delete();
        wr_flush = 1;
        @(posedge clk);
    endtask

    // -------------------------------------------------------------------------
    // Операции с EEPROM: все сверяются с эталоном
    // -------------------------------------------------------------------------
    int exp_cycles = 0;                             // сколько циклов записи ожидается

    function automatic logic [6:0] dev(int addr);
        return 7'h50 + 7'(addr >> 8);
    endfunction

    // запись addr, data...; STOP. Возвращает признак ACK адреса
    task automatic ee_write(input int addr, input bytes_t data, output bit acked);
        Res     r;
        bytes_t wd;
        wd.push_back(addr[7:0]);
        foreach (data[i]) wd.push_back(data[i]);
        txn(dev(addr), 0, wd.size(), 1, wd, r);
        acked = !r.nack_addr && !r.nack_data;
        if (acked) begin
            if (ref_m.write(addr >> 8, wd) > 0) exp_cycles++;
        end
    endtask

    // опрос ACK: пока адрес не подтверждается. Возвращает число NACK
    task automatic ee_poll(input int addr, output int n_nack);
        Res     r;
        bytes_t none;
        n_nack = 0;
        do begin
            txn(dev(addr), 0, 0, 1, none, r);
            if (r.nack_addr) n_nack++;
            if (n_nack > 200) begin
                sb.err("polling does not finish");
                break;
            end
        end while (r.nack_addr);
    endtask

    task automatic ee_write_wait(input int addr, input bytes_t data);
        bit acked;
        int n_nack;
        ee_write(addr, data, acked);
        if (!acked) sb.err($sformatf("write at %0d: no ACK", addr));
        ee_poll(addr, n_nack);
    endtask

    // чтение n байт с адреса addr: запись адреса без STOP, повторный START, чтение
    task automatic ee_read(input int addr, input int n, output bytes_t got);
        Res     r;
        bytes_t wd, none;
        wd.push_back(addr[7:0]);
        txn(dev(addr), 0, 1, 0, wd, r);
        if (r.nack_addr || r.nack_data) sb.err($sformatf("read at %0d: pointer write NACK", addr));
        void'(ref_m.write(addr >> 8, wd));
        txn(dev(addr), 1, n, 1, none, r);
        if (r.nack_addr) sb.err($sformatf("read at %0d: no ACK on read address", addr));
        got = r.rd;
    endtask

    task automatic check_read(input string name, input int addr, input int n);
        bytes_t got, exp_d;
        ee_read(addr, n, got);
        exp_d = ref_m.read(addr >> 8, n);
        if (got.size() != exp_d.size() || got != exp_d)
            sb.err($sformatf("%s: read %0d bytes at %0d: got %p, expected %p", name, n, addr, got, exp_d));
    endtask

    // чтение с текущего адреса
    task automatic check_read_cur(input string name, input int blk, input int n);
        Res     r;
        bytes_t none, exp_d;
        txn(7'h50 + 7'(blk), 1, n, 1, none, r);
        exp_d = ref_m.read(blk, n);
        if (r.nack_addr) sb.err($sformatf("%s: no ACK", name));
        else if (r.rd != exp_d)
            sb.err($sformatf("%s: current read got %p, expected %p", name, r.rd, exp_d));
    endtask

    task automatic check_mem(input string name);
        for (int i = 0; i < 1024; i++)
            if (model.mem[i] !== ref_m.m[i]) begin
                sb.err($sformatf("%s: mem[%0d] = %h, expected %h", name, i, model.mem[i], ref_m.m[i]));
                break;
            end
    endtask

    function automatic bytes_t rnd_bytes(int n);
        bytes_t d;
        repeat (n) d.push_back($urandom);
        return d;
    endfunction

    function automatic bytes_t seq_bytes(int n, logic [7:0] first);
        bytes_t d;
        for (int i = 0; i < n; i++) d.push_back(first + 8'(i));
        return d;
    endfunction

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        Res     r;
        bytes_t d, none, got, wd;
        bit     acked;
        int     n_nack, c0, addr, n;
        $timeformat(-6, 2, " us", 0);
        #(CLK_NS * 5);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // ---------------------------------------------------------------- 1
        check_mem("T1 fresh");
        for (int b = 0; b < 4; b++) begin
            check_read($sformatf("T1 block %0d", b), b * 256 + 17, 6);
            check_read($sformatf("T1 block %0d end", b), b * 256 + 250, 6);
        end
        if (wr_cycles !== 16'd0) sb.err("T1: write cycles after reads");

        // ---------------------------------------------------------------- 2
        d.delete();  d.push_back(8'h5A);
        ee_write(16'h123, d, acked);
        if (!acked) sb.err("T2: write not acknowledged");
        // сразу после STOP: микросхема занята
        txn(7'h51, 0, 0, 1, none, r);
        if (!r.nack_addr) sb.err("T2: ACK during the write cycle (probe)");
        wd.delete();  wd.push_back(8'h00);
        txn(7'h50, 0, 1, 0, wd, r);
        if (!r.nack_addr) sb.err("T2: write pointer accepted during the write cycle");
        none.delete();
        txn(7'h50, 1, 1, 1, none, r);
        if (!r.nack_addr) sb.err("T2: read accepted during the write cycle");
        if (!model_busy) sb.err("T2: busy = 0 right after the write");
        ee_poll(16'h123, n_nack);
        if (n_nack < 30) sb.err($sformatf("T2: only %0d NACK in polling for a %0d us write cycle", n_nack, WRITE_US));
        if (n_busy_rises != 1) sb.err($sformatf("T2: %0d write cycles started, expected 1", n_busy_rises));
        if (t_fall - t_rise < WRITE_US * 1000.0 - 40 || t_fall - t_rise > WRITE_US * 1000.0 + 40)
            sb.err($sformatf("T2: write cycle %0t us, expected %0d us", t_fall - t_rise, WRITE_US));
        if (wr_cycles !== 16'd1) sb.err($sformatf("T2: wr_cycles = %0d, expected 1", wr_cycles));
        if (model.mem[16'h123] !== 8'h5A) sb.err($sformatf("T2: mem[123] = %h, expected 5A", model.mem[16'h123]));
        check_mem("T2 mem");
        check_read("T2 readback", 16'h123, 1);
        check_read("T2 neighbours", 16'h120, 8);

        // ---------------------------------------------------------------- 3
        ee_write_wait(16'h00C, seq_bytes(5, 8'h11));            // 00C 00D 00E 00F, затем 000
        check_mem("T3 wrap 5 bytes");
        ee_write_wait(16'h0F3, seq_bytes(16, 8'h20));           // ровно страница из середины
        check_mem("T3 page 16 bytes from the middle");
        ee_write_wait(16'h235, seq_bytes(20, 8'h40));           // 20 байт: хвост затирает начало
        check_mem("T3 20 bytes");
        ee_write_wait(16'h3F0, seq_bytes(16, 8'h80));           // последняя страница памяти
        check_mem("T3 last page");
        ee_write_wait(16'h0FF, seq_bytes(1, 8'hC3));            // последний байт блока 0
        ee_write_wait(16'h100, seq_bytes(1, 8'hC4));            // первый байт блока 1
        check_mem("T3 block edge");
        if (wr_cycles !== 16'(exp_cycles))
            sb.err($sformatf("T3: wr_cycles = %0d, expected %0d (one cycle per write transaction)", wr_cycles, exp_cycles));
        check_read("T3 readback page", 16'h235, 16);

        // ---------------------------------------------------------------- 4
        for (int i = 0; i < 16; i++) ee_write_wait(16'h0F0 + i * 2, seq_bytes(2, 8'(i * 16 + 1)));
        check_read("T4 random read", 16'h0F5, 1);
        check_read("T4 random read", 16'h3FB, 4);               // 3FB..3FF
        check_read("T4 across blocks 0->1", 16'h0FC, 8);
        check_read("T4 across blocks 2->3", 16'h2FE, 5);
        check_read("T4 wrap 1023->0", 16'h3FD, 8);
        check_read_cur("T4 current after read", 3, 4);          // продолжение с адреса после чтения
        check_read_cur("T4 current again", 3, 2);
        d = seq_bytes(3, 8'h70);
        ee_write_wait(16'h148, d);
        check_read_cur("T4 current after write", 1, 2);
        check_read("T4 long read", 16'h000, 300);

        // ---------------------------------------------------------------- 5
        c0 = exp_cycles;
        wd.delete();  wd.push_back(8'h33);
        txn(7'h50, 0, 1, 1, wd, r);                              // адрес ячейки без данных
        void'(ref_m.write(0, wd));
        if (r.nack_addr || r.nack_data) sb.err("T5: address-only write NACK");
        repeat (20) @(posedge clk);
        if (model_busy) sb.err("T5: write cycle after an address-only write");
        check_read_cur("T5 pointer after address-only write", 0, 2);
        // запись, прерванная повторным START (чтение с текущего адреса вместо STOP)
        wd.delete();  wd.push_back(8'h40);  wd.push_back(8'hDE);  wd.push_back(8'hAD);
        txn(7'h50, 0, 3, 0, wd, r);
        void'(ref_m.write(0, wd, 0));
        if (r.nack_addr || r.nack_data) sb.err("T5: aborted write NACK");
        none.delete();
        txn(7'h50, 1, 2, 1, none, r);
        void'(ref_m.read(0, 2));
        repeat (20) @(posedge clk);
        if (model_busy) sb.err("T5: write cycle after a write without STOP");
        if (wr_cycles !== 16'(c0)) sb.err($sformatf("T5: wr_cycles = %0d, expected %0d", wr_cycles, c0));
        check_mem("T5 memory unchanged");

        // ---------------------------------------------------------------- 6
        none.delete();
        for (int a = 7'h4E; a <= 7'h55; a++) begin
            txn(7'(a), 0, 0, 1, none, r);
            if ((a >= 7'h50 && a <= 7'h53) == r.nack_addr)
                sb.err($sformatf("T6: address %h: nack_addr = %b", a, r.nack_addr));
        end
        ee_write_wait(16'h3A5, seq_bytes(2, 8'hBB));            // через адрес 0x53
        if (model.mem[16'h3A5] !== 8'hBB || model.mem[16'h3A6] !== 8'hBC)
            sb.err("T6: write through address 0x53 did not reach block 3");
        present = 1'b0;
        for (int a = 7'h50; a <= 7'h53; a++) begin
            txn(7'(a), 0, 0, 1, none, r);
            if (!r.nack_addr) sb.err($sformatf("T6: address %h answers with present = 0", a));
        end
        present = 1'b1;
        repeat (10) @(posedge clk);
        txn(7'h50, 0, 0, 1, none, r);
        if (r.nack_addr) sb.err("T6: no ACK after present = 1");
        check_mem("T6 memory");

        // ---------------------------------------------------------------- 7
        for (int i = 0; i < 25; i++) begin
            addr = $urandom_range(0, 1023);
            n    = $urandom_range(1, 20);
            ee_write_wait(addr, rnd_bytes(n));
            check_read($sformatf("T7 write %0d", i), $urandom_range(0, 1023), $urandom_range(1, 40));
        end
        check_mem("T7 mem");
        if (wr_cycles !== 16'(exp_cycles))
            sb.err($sformatf("T7: wr_cycles = %0d, expected %0d", wr_cycles, exp_cycles));
        check_read("T7 full dump", 0, 1024);

        repeat (10) @(posedge clk);
        if (sb.errors == 0) $display("TEST PASSED");
        else                $display("TEST FAILED: %0d errors", sb.errors);
        $finish;
    end

endmodule
