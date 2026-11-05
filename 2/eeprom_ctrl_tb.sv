// =============================================================================
// Тестбенч для eeprom_ctrl: запись с ACK-polling, чтение, ошибки; 50 МГц, SCL 100 кГц
//
// Три окружения ee_ctrl_env (контроллер + i2c_master + своя шина):
//   0 - независимая модель i2c_slave_model (тестбенчевая, цикл записи 1 мс, страница 16);
//   1 - имитатор eeprom_model (синтезируемый, цикл записи 1 мс);
//   2 - «медленная» микросхема (i2c_slave_model, цикл записи 30 мс) для предела опросов.
// Окружения 0 и 1 выполняют один и тот же сценарий параллельно. Эталон - класс RefMem
// (память, текущий адрес, буфер страницы).
// Проверки:
//   C1. WR_BYTE: ok, число опросов ACK соответствует циклу записи (1 мс - 4..12 опросов),
//       память совпадает с эталоном, один цикл записи на операцию
//   C2. WR_WORD: старший байт по меньшему адресу; слово в середине страницы - один цикл
//       записи; слово на границе страницы (первый байт - последний в странице) - два цикла;
//       граница блока 256 байт (адрес устройства меняется); конец памяти 1023 -> 0
//   C3. RD_BYTE, RD_SEQ (1, 2, 16 и 100 байт, через границу блока и конец памяти),
//       RD_CUR (продолжение после чтения и после записи): данные и rd_last, случайные
//       паузы data_ready (data_valid и data не меняются, пока нет data_ready)
//   C4. Микросхемы нет (absent / present = 0): запись и чтение дают op_nack, опросов нет;
//       после возвращения всё работает
//   C5. Окружение 2: запись не заканчивается за POLL_MAX опросов - op_poll, op_polls = POLL_MAX;
//       после окончания цикла записи следующая операция выполняется
//   C6. Окружение 0: растяжка SCL дольше предела - op_bus для записи и чтения; восстановление
//   C7. Окружение 0: NACK на байт адреса ячейки и на байт данных - op_nack, память не меняется
//   C8. 60 случайных операций (запись байта и слова, чтение разной длины) в окружениях 0 и 1;
//       после каждой проверка по эталону; в конце сравнение всей памяти
// Общие проверки: op_done - один такт, ровно один из флагов ok / nack / bus / poll,
// req_ready = 0, пока операция выполняется.
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Окружение: контроллер + мастер на своей шине (только для тестбенча)
// -----------------------------------------------------------------------------
module ee_ctrl_env (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        req_valid,
    output logic        req_ready,
    input  logic [2:0]  req_op,
    input  logic [9:0]  req_addr,
    input  logic [15:0] req_data,
    input  logic [7:0]  req_len,
    output logic        data_valid,
    input  logic        data_ready,
    output logic [7:0]  data,
    output logic        data_last,
    output logic        op_done,
    output logic        op_ok,
    output logic        op_nack,
    output logic        op_bus,
    output logic        op_poll,
    output logic [7:0]  op_polls,
    output logic        polling,
    output logic        ev_start,
    output logic        ev_stop,
    inout  tri1         scl,
    inout  tri1         sda
);

    localparam int LEN_W = 8;

    logic             cmd_valid, cmd_ready, cmd_read, cmd_stop;
    logic [6:0]       cmd_addr;
    logic [LEN_W-1:0] cmd_len;
    logic [7:0]       wr_data, rd_data;
    logic             wr_valid, wr_ready, rd_valid, rd_ready, rd_last;
    logic             done, res_nack_addr, res_nack_data, res_timeout, res_arb;
    logic [LEN_W-1:0] res_len;

    eeprom_ctrl #(
        .LEN_W      (LEN_W),
        .ADDR_W     (10),
        .DEV_ADDR   (7'h50),
        .PAGE_BYTES (16),
        .POLL_MAX   (100)
    ) u_ctrl (
        .clk (clk), .rst_n (rst_n),
        .req_valid (req_valid), .req_ready (req_ready), .req_op (req_op), .req_addr (req_addr),
        .req_data (req_data), .req_len (req_len),
        .data_valid (data_valid), .data_ready (data_ready), .data (data), .data_last (data_last),
        .op_done (op_done), .op_ok (op_ok), .op_nack (op_nack), .op_bus (op_bus), .op_poll (op_poll),
        .op_polls (op_polls), .polling (polling),
        .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_addr (cmd_addr), .cmd_read (cmd_read),
        .cmd_len (cmd_len), .cmd_stop (cmd_stop),
        .wr_data (wr_data), .wr_valid (wr_valid), .wr_ready (wr_ready),
        .rd_data (rd_data), .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_last (rd_last),
        .done (done), .res_nack_addr (res_nack_addr), .res_nack_data (res_nack_data),
        .res_timeout (res_timeout), .res_arb (res_arb)
    );

    i2c_master #(
        .CLK_FREQ   (50_000_000),
        .SCL_FREQ   (100_000),
        .TIMEOUT_US (1_000),
        .LEN_W      (LEN_W)
    ) u_master (
        .clk (clk), .rst_n (rst_n),
        .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_addr (cmd_addr), .cmd_read (cmd_read),
        .cmd_len (cmd_len), .cmd_stop (cmd_stop),
        .wr_data (wr_data), .wr_valid (wr_valid), .wr_ready (wr_ready),
        .rd_data (rd_data), .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_last (rd_last),
        .done (done), .res_len (res_len), .res_nack_addr (res_nack_addr), .res_nack_data (res_nack_data),
        .res_timeout (res_timeout), .res_arb (res_arb),
        .ev_start (ev_start), .ev_stop (ev_stop), .ev_ack (), .ev_nack (),
        .i2c_scl (scl), .i2c_sda (sda)
    );

endmodule

// -----------------------------------------------------------------------------
module eeprom_ctrl_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  N_ENV    = 3;
    localparam int  POLL_MAX = 100;

    localparam int OP_WR_BYTE = 0,
                   OP_WR_WORD = 1,
                   OP_RD_BYTE = 2,
                   OP_RD_SEQ  = 3,
                   OP_RD_CUR  = 4;

    typedef logic [7:0] bytes_t[$];

    // эталон AT24C08: память 1024 байта, текущий адрес, буфер страницы 16 байт
    class RefMem;
        logic [7:0] m [1024];
        int         ptr = 0;
        int         cycles = 0;                 // сколько циклов записи ожидается

        function new();
            foreach (m[i]) m[i] = 8'hFF;
        endfunction

        function void addr_phase(int blk);
            ptr = blk * 256 + (ptr % 256);
        endfunction

        // d[0] - младший байт адреса, остальные - данные
        function void write(int blk, bytes_t d);
            logic [7:0] pb [16];
            bit         pv [16];
            int         base;
            addr_phase(blk);
            ptr  = blk * 256 + d[0];
            base = (ptr / 16) * 16;
            foreach (pv[i]) pv[i] = 0;
            for (int i = 1; i < d.size(); i++) begin
                pb[ptr % 16] = d[i];
                pv[ptr % 16] = 1;
                ptr = base + ((ptr + 1) % 16);
            end
            if (d.size() > 1) begin
                for (int k = 0; k < 16; k++)
                    if (pv[k]) m[base + k] = pb[k];
                cycles++;
            end
        endfunction

        function void set_ptr(int addr);
            bytes_t d;
            d.push_back(addr[7:0]);
            write(addr >> 8, d);
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

        // слово: старший байт по меньшему адресу; на границе страницы - два цикла записи
        function void write_word(int addr, logic [15:0] w);
            bytes_t d;
            if (addr % 16 == 15) begin
                int a2;
                d.push_back(addr[7:0]);  d.push_back(w[15:8]);
                write(addr >> 8, d);
                a2 = (addr + 1) % 1024;
                d.delete();
                d.push_back(a2[7:0]);  d.push_back(w[7:0]);
                write(a2 >> 8, d);
            end else begin
                d.push_back(addr[7:0]);  d.push_back(w[15:8]);  d.push_back(w[7:0]);
                write(addr >> 8, d);
            end
        endfunction
    endclass

    class Scoreboard;
        int errors = 0;

        function void err(string msg);
            errors++;
            if (errors <= 25) $display("[%0t] ERROR: %s", $time, msg);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы
    // -------------------------------------------------------------------------
    logic              clk = 1'b0, rst_n = 1'b0;
    logic [N_ENV-1:0]  req_valid = '0, req_ready, data_valid, data_last, op_done;
    logic [N_ENV-1:0]  data_ready = '1, op_ok, op_nack, op_bus, op_poll, polling;
    logic [2:0]        req_op   [N_ENV];
    logic [9:0]        req_addr [N_ENV];
    logic [15:0]       req_data [N_ENV];
    logic [7:0]        req_len  [N_ENV];
    logic [7:0]        data     [N_ENV];
    logic [7:0]        op_polls [N_ENV];
    logic [N_ENV-1:0]  ev_start, ev_stop;
    tri1               scl [N_ENV];
    tri1               sda [N_ENV];

    logic              present1 = 1'b1;
    logic              model1_busy, model1_sda_low;
    logic [15:0]       model1_cycles;

    always #(CLK_NS / 2) clk = ~clk;

    ee_ctrl_env env0 (
        .clk (clk), .rst_n (rst_n),
        .req_valid (req_valid[0]), .req_ready (req_ready[0]), .req_op (req_op[0]), .req_addr (req_addr[0]),
        .req_data (req_data[0]), .req_len (req_len[0]),
        .data_valid (data_valid[0]), .data_ready (data_ready[0]), .data (data[0]), .data_last (data_last[0]),
        .op_done (op_done[0]), .op_ok (op_ok[0]), .op_nack (op_nack[0]), .op_bus (op_bus[0]),
        .op_poll (op_poll[0]), .op_polls (op_polls[0]), .polling (polling[0]),
        .ev_start (ev_start[0]), .ev_stop (ev_stop[0]), .scl (scl[0]), .sda (sda[0])
    );
    ee_ctrl_env env1 (
        .clk (clk), .rst_n (rst_n),
        .req_valid (req_valid[1]), .req_ready (req_ready[1]), .req_op (req_op[1]), .req_addr (req_addr[1]),
        .req_data (req_data[1]), .req_len (req_len[1]),
        .data_valid (data_valid[1]), .data_ready (data_ready[1]), .data (data[1]), .data_last (data_last[1]),
        .op_done (op_done[1]), .op_ok (op_ok[1]), .op_nack (op_nack[1]), .op_bus (op_bus[1]),
        .op_poll (op_poll[1]), .op_polls (op_polls[1]), .polling (polling[1]),
        .ev_start (ev_start[1]), .ev_stop (ev_stop[1]), .scl (scl[1]), .sda (sda[1])
    );
    ee_ctrl_env env2 (
        .clk (clk), .rst_n (rst_n),
        .req_valid (req_valid[2]), .req_ready (req_ready[2]), .req_op (req_op[2]), .req_addr (req_addr[2]),
        .req_data (req_data[2]), .req_len (req_len[2]),
        .data_valid (data_valid[2]), .data_ready (data_ready[2]), .data (data[2]), .data_last (data_last[2]),
        .op_done (op_done[2]), .op_ok (op_ok[2]), .op_nack (op_nack[2]), .op_bus (op_bus[2]),
        .op_poll (op_poll[2]), .op_polls (op_polls[2]), .polling (polling[2]),
        .ev_start (ev_start[2]), .ev_stop (ev_stop[2]), .scl (scl[2]), .sda (sda[2])
    );

    i2c_slave_model #(
        .ADDR           (7'h50),
        .ADDR_MASK      (7'h7C),
        .MEM_BITS       (10),
        .BLOCK_BITS     (2),
        .PAGE_SIZE      (16),
        .WRITE_CYCLE_NS (1_000_000.0)
    ) slave0 (.scl (scl[0]), .sda (sda[0]));

    eeprom_model #(
        .CLK_FREQ   (CLK_FREQ),
        .ADDR       (7'h50),
        .WRITE_US   (1000)
    ) model1 (
        .clk (clk), .rst_n (rst_n), .present (present1), .busy (model1_busy), .wr_cycles (model1_cycles),
        .scl_in (scl[1]), .sda_in (sda[1]), .sda_low (model1_sda_low)
    );
    assign sda[1] = model1_sda_low ? 1'b0 : 1'bz;

    i2c_slave_model #(
        .ADDR           (7'h50),
        .ADDR_MASK      (7'h7C),
        .MEM_BITS       (10),
        .BLOCK_BITS     (2),
        .PAGE_SIZE      (16),
        .WRITE_CYCLE_NS (30_000_000.0)
    ) slave2 (.scl (scl[2]), .sda (sda[2]));

    Scoreboard sb = new();
    RefMem     refm [N_ENV];

    // -------------------------------------------------------------------------
    // Мониторы
    // -------------------------------------------------------------------------
    int     done_cnt [N_ENV];
    int     last_cnt [N_ENV];
    bytes_t rdq      [N_ENV];
    bit     bp       [N_ENV];                       // случайные паузы data_ready

    logic [N_ENV-1:0] done_q = '0, valid_q = '0, ready_q = '1;
    logic [7:0]       data_q [N_ENV];
    int               n_start [N_ENV];              // сколько START и STOP было на шине за операцию
    int               n_stop  [N_ENV];

    always @(posedge clk) begin
        for (int i = 0; i < N_ENV; i++) begin
            if (ev_start[i]) n_start[i]++;
            if (ev_stop[i])  n_stop[i]++;
            if (op_done[i]) begin
                done_cnt[i]++;
                if (done_q[i]) sb.err($sformatf("env %0d: op_done is wider than 1 clock", i));
                if ((op_ok[i] + op_nack[i] + op_bus[i] + op_poll[i]) != 1)
                    sb.err($sformatf("env %0d: flags ok/nack/bus/poll = %b%b%b%b at op_done", i,
                                     op_ok[i], op_nack[i], op_bus[i], op_poll[i]));
            end
            if (data_valid[i] && data_ready[i]) begin
                rdq[i].push_back(data[i]);
                if (data_last[i]) last_cnt[i]++;
            end
            if (rst_n && valid_q[i] && !ready_q[i] && (!data_valid[i] || data[i] !== data_q[i]))
                sb.err($sformatf("env %0d: data_valid / data changed before data_ready", i));
            data_ready[i] <= bp[i] ? ($urandom_range(0, 2) != 0) : 1'b1;
        end
        done_q  <= op_done;
        valid_q <= data_valid;
        ready_q <= data_ready;
        for (int i = 0; i < N_ENV; i++) data_q[i] <= data[i];
    end

    // -------------------------------------------------------------------------
    // Одна операция: запрос, ожидание op_done, итог
    // -------------------------------------------------------------------------
    bit [N_ENV-1:0] g_ok, g_nack, g_bus, g_poll;
    int             g_polls [N_ENV];
    int             g_starts [N_ENV];
    int             g_stops  [N_ENV];

    task automatic run_op(input int idx, input int op, input int addr, input logic [15:0] wdata, input int len);
        int d0, guard;
        d0 = done_cnt[idx];
        rdq[idx].delete();
        last_cnt[idx] = 0;
        @(posedge clk);
        n_start[idx] = 0;
        n_stop[idx]  = 0;
        req_op[idx]   <= 3'(op);
        req_addr[idx] <= 10'(addr);
        req_data[idx] <= wdata;
        req_len[idx]  <= 8'(len);
        req_valid[idx] <= 1'b1;
        do @(posedge clk); while (!req_ready[idx]);
        req_valid[idx] <= 1'b0;
        repeat (2) @(posedge clk);
        if (req_ready[idx]) sb.err($sformatf("env %0d: req_ready = 1 while the operation runs", idx));
        guard = 0;
        while (done_cnt[idx] == d0 && guard < 5_000_000) begin
            @(posedge clk);
            guard++;
        end
        if (guard >= 5_000_000) sb.err($sformatf("env %0d: operation %0d does not finish", idx, op));
        repeat (2) @(posedge clk);
        g_ok[idx]    = op_ok[idx];
        g_nack[idx]  = op_nack[idx];
        g_bus[idx]   = op_bus[idx];
        g_poll[idx]  = op_poll[idx];
        g_polls[idx] = op_polls[idx];
        g_starts[idx] = n_start[idx];
        g_stops[idx]  = n_stop[idx];
    endtask

    // состав обмена на шине: запись - по START и STOP на кусок и на каждый опрос (и ещё один,
    // подтверждённый); чтение - запись адреса с повторным START (RD_BYTE, RD_SEQ) и чтение
    task automatic check_bus(input string name, input int idx, input int starts, input int stops);
        if (g_starts[idx] != starts || g_stops[idx] != stops)
            sb.err($sformatf("%s env %0d: bus has %0d START and %0d STOP, expected %0d and %0d",
                             name, idx, g_starts[idx], g_stops[idx], starts, stops));
    endtask

    task automatic expect_flags(input string name, input int idx, input bit ok, input bit nack, input bit bus, input bit poll);
        if (g_ok[idx] !== ok || g_nack[idx] !== nack || g_bus[idx] !== bus || g_poll[idx] !== poll)
            sb.err($sformatf("%s env %0d: flags ok/nack/bus/poll = %b%b%b%b, expected %b%b%b%b", name, idx,
                             g_ok[idx], g_nack[idx], g_bus[idx], g_poll[idx], ok, nack, bus, poll));
    endtask

    // число опросов для k циклов записи по 1 мс: у одного опроса около 120 мкс
    task automatic check_polls(input string name, input int idx, input int cycles);
        if (g_polls[idx] < 4 * cycles || g_polls[idx] > 12 * cycles)
            sb.err($sformatf("%s env %0d: %0d polls for %0d write cycles of 1 ms", name, idx, g_polls[idx], cycles));
    endtask

    task automatic wr_byte(input string name, input int idx, input int addr, input logic [7:0] v);
        bytes_t d;
        run_op(idx, OP_WR_BYTE, addr, {8'h00, v}, 0);
        expect_flags(name, idx, 1, 0, 0, 0);
        check_polls(name, idx, 1);
        check_bus(name, idx, 2 + g_polls[idx], 2 + g_polls[idx]);
        d.push_back(addr[7:0]);  d.push_back(v);
        refm[idx].write(addr >> 8, d);
    endtask

    task automatic wr_word(input string name, input int idx, input int addr, input logic [15:0] w);
        int c0;
        c0 = refm[idx].cycles;
        run_op(idx, OP_WR_WORD, addr, w, 0);
        expect_flags(name, idx, 1, 0, 0, 0);
        refm[idx].write_word(addr, w);
        check_polls(name, idx, refm[idx].cycles - c0);
        check_bus(name, idx, 2 * (refm[idx].cycles - c0) + g_polls[idx], 2 * (refm[idx].cycles - c0) + g_polls[idx]);
    endtask

    task automatic rd_check(input string name, input int idx, input int op, input int addr, input int len);
        bytes_t exp_d;
        int     n;
        n = (op == OP_RD_BYTE) ? 1 : (len == 0) ? 1 : len;
        if (op != OP_RD_CUR) refm[idx].set_ptr(addr);
        exp_d = refm[idx].read(addr >> 8, n);
        run_op(idx, op, addr, '0, len);
        expect_flags(name, idx, 1, 0, 0, 0);
        if (rdq[idx] != exp_d)
            sb.err($sformatf("%s env %0d: read %0d bytes at %0d: got %p, expected %p", name, idx, n, addr, rdq[idx], exp_d));
        if (last_cnt[idx] != 1)
            sb.err($sformatf("%s env %0d: data_last seen %0d times, expected 1", name, idx, last_cnt[idx]));
        if (g_polls[idx] != 0) sb.err($sformatf("%s env %0d: %0d polls in a read", name, idx, g_polls[idx]));
        check_bus(name, idx, (op == OP_RD_CUR) ? 1 : 2, 1);
    endtask

    task automatic cmp_mem(input string name, input int idx);
        for (int i = 0; i < 1024; i++) begin
            logic [7:0] got;
            got = (idx == 0) ? slave0.mem[i] : (idx == 1) ? model1.mem[i] : slave2.mem[i];
            if (got !== refm[idx].m[i]) begin
                sb.err($sformatf("%s env %0d: mem[%0d] = %h, expected %h", name, idx, i, got, refm[idx].m[i]));
                break;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Общий сценарий окружений 0 и 1
    // -------------------------------------------------------------------------
    task automatic common(input int idx);
        int  op, addr, len;
        logic [15:0] w;

        // ---------------------------------------------------------------- C1
        wr_byte("C1", idx, 16'h000, 8'hA5);
        wr_byte("C1", idx, 16'h2FF, 8'h3C);
        wr_byte("C1", idx, 16'h155, 8'h81);
        cmp_mem("C1", idx);

        // ---------------------------------------------------------------- C2
        wr_word("C2 mid-page", idx, 16'h010, 16'h1960);
        wr_word("C2 odd addr", idx, 16'h1F1, 16'hBEEF);
        wr_word("C2 page edge", idx, 16'h01F, 16'hCAFE);
        wr_word("C2 block edge", idx, 16'h0FF, 16'h1234);
        wr_word("C2 memory end", idx, 16'h3FF, 16'h9ABC);
        wr_word("C2 next page", idx, 16'h030, 16'h5566);
        cmp_mem("C2", idx);
        if (idx == 1 && model1_cycles !== 16'(refm[idx].cycles))
            sb.err($sformatf("C2: model wr_cycles = %0d, expected %0d", model1_cycles, refm[idx].cycles));

        // ---------------------------------------------------------------- C3
        bp[idx] = 1'b0;
        rd_check("C3 byte", idx, OP_RD_BYTE, 16'h000, 0);
        rd_check("C3 byte", idx, OP_RD_BYTE, 16'h2FF, 0);
        rd_check("C3 seq 1", idx, OP_RD_SEQ, 16'h010, 1);
        rd_check("C3 seq 2", idx, OP_RD_SEQ, 16'h010, 2);
        rd_check("C3 seq 16", idx, OP_RD_SEQ, 16'h1F0, 16);
        rd_check("C3 seq edge", idx, OP_RD_SEQ, 16'h0FC, 8);
        rd_check("C3 seq end", idx, OP_RD_SEQ, 16'h3FC, 8);
        rd_check("C3 cur", idx, OP_RD_CUR, 16'h000, 4);
        rd_check("C3 cur", idx, OP_RD_CUR, 16'h000, 3);
        rd_check("C3 seq 100", idx, OP_RD_SEQ, 16'h040, 100);
        bp[idx] = 1'b1;
        rd_check("C3 seq bp", idx, OP_RD_SEQ, 16'h000, 40);
        rd_check("C3 byte bp", idx, OP_RD_BYTE, 16'h01F, 0);
        bp[idx] = 1'b0;
        wr_word("C3 write", idx, 16'h200, 16'h7788);
        rd_check("C3 cur after write", idx, OP_RD_CUR, 16'h200, 2);

        // ---------------------------------------------------------------- C8
        for (int i = 0; i < 60; i++) begin
            op   = $urandom_range(0, 4);
            addr = $urandom_range(0, 1023);
            len  = $urandom_range(1, 50);
            w    = $urandom;
            bp[idx] = ($urandom_range(0, 3) == 0);
            case (op)
                OP_WR_BYTE: wr_byte($sformatf("C8 %0d", i), idx, addr, w[7:0]);
                OP_WR_WORD: wr_word($sformatf("C8 %0d", i), idx, addr, w);
                OP_RD_CUR:  rd_check($sformatf("C8 %0d", i), idx, OP_RD_CUR, addr & 16'h300, len);
                default:    rd_check($sformatf("C8 %0d", i), idx, op, addr, len);
            endcase
        end
        bp[idx] = 1'b0;
        cmp_mem("C8", idx);
        if (idx == 1 && model1_cycles !== 16'(refm[idx].cycles))
            sb.err($sformatf("C8: model wr_cycles = %0d, expected %0d", model1_cycles, refm[idx].cycles));
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        for (int i = 0; i < N_ENV; i++) begin
            refm[i] = new();
            done_cnt[i] = 0;  last_cnt[i] = 0;  bp[i] = 0;
            n_start[i] = 0;   n_stop[i] = 0;
        end
        $timeformat(-6, 2, " us", 0);
        for (int i = 0; i < 1024; i++) begin        // новая микросхема - 0xFF (i2c_slave_model стартует с 0x00)
            slave0.mem[i] = 8'hFF;
            slave2.mem[i] = 8'hFF;
        end
        #(CLK_NS * 5);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // C1, C2, C3, C8 в двух окружениях параллельно
        fork
            common(0);
            common(1);
        join

        // ---------------------------------------------------------------- C4
        slave0.absent = 1'b1;
        present1      = 1'b0;
        for (int idx = 0; idx < 2; idx++) begin
            run_op(idx, OP_WR_BYTE, 16'h111, 16'h0042, 0);
            expect_flags("C4 write", idx, 0, 1, 0, 0);
            if (g_polls[idx] != 0) sb.err("C4: polling after a NACK on write");
            run_op(idx, OP_WR_WORD, 16'h112, 16'h4243, 0);
            expect_flags("C4 word", idx, 0, 1, 0, 0);
            run_op(idx, OP_RD_BYTE, 16'h111, '0, 0);
            expect_flags("C4 read", idx, 0, 1, 0, 0);
            run_op(idx, OP_RD_SEQ, 16'h111, '0, 4);
            expect_flags("C4 seq", idx, 0, 1, 0, 0);
            run_op(idx, OP_RD_CUR, 16'h111, '0, 4);
            expect_flags("C4 cur", idx, 0, 1, 0, 0);
            if (rdq[idx].size() != 0) sb.err("C4: data from an absent chip");
        end
        slave0.absent = 1'b0;
        present1      = 1'b1;
        repeat (10) @(posedge clk);
        for (int idx = 0; idx < 2; idx++) begin
            wr_byte("C4 recovery", idx, 16'h111, 8'h42);
            rd_check("C4 recovery", idx, OP_RD_BYTE, 16'h111, 0);
        end
        cmp_mem("C4", 0);
        cmp_mem("C4", 1);

        // ---------------------------------------------------------------- C5
        run_op(2, OP_WR_BYTE, 16'h070, 16'h0099, 0);
        expect_flags("C5", 2, 0, 0, 0, 1);
        if (g_polls[2] != POLL_MAX) sb.err($sformatf("C5: %0d polls, expected POLL_MAX = %0d", g_polls[2], POLL_MAX));
        begin
            bytes_t d;
            d.push_back(8'h70);  d.push_back(8'h99);
            refm[2].write(0, d);                    // данные записаны, просто контроллер не дождался
        end
        #(30_000_000);                              // цикл записи закончился
        rd_check("C5 after", 2, OP_RD_BYTE, 16'h070, 0);      // в этом окружении цикл 30 мс: пишем только раз
        cmp_mem("C5", 2);

        // ---------------------------------------------------------------- C6
        slave0.stretch_ns = 1_500_000;
        run_op(0, OP_RD_BYTE, 16'h010, '0, 0);
        expect_flags("C6 read", 0, 0, 0, 1, 0);
        slave0.stretch_ns = 0;
        #(3_000_000);
        slave0.stretch_ns = 1_500_000;
        run_op(0, OP_WR_BYTE, 16'h013, 16'h0011, 0);
        expect_flags("C6 write", 0, 0, 0, 1, 0);
        slave0.stretch_ns = 0;
        #(3_000_000);
        wr_byte("C6 recovery", 0, 16'h013, 8'h11);
        rd_check("C6 recovery", 0, OP_RD_BYTE, 16'h013, 0);
        cmp_mem("C6", 0);

        // ---------------------------------------------------------------- C7
        slave0.wr_limit = 0;                        // NACK уже на байт адреса ячейки
        run_op(0, OP_WR_BYTE, 16'h020, 16'h0077, 0);
        expect_flags("C7 address byte", 0, 0, 1, 0, 0);
        run_op(0, OP_RD_BYTE, 16'h020, '0, 0);
        expect_flags("C7 read pointer", 0, 0, 1, 0, 0);
        slave0.wr_limit = 1;                        // адрес принят, байт данных - NACK
        run_op(0, OP_WR_BYTE, 16'h020, 16'h0077, 0);
        expect_flags("C7 data byte", 0, 0, 1, 0, 0);
        if (g_polls[0] != 0) sb.err("C7: polling after NACK on data");
        slave0.wr_limit = 1_000_000;
        cmp_mem("C7 memory unchanged", 0);
        wr_byte("C7 recovery", 0, 16'h020, 8'h77);
        rd_check("C7 recovery", 0, OP_RD_BYTE, 16'h020, 0);
        cmp_mem("C7", 0);

        repeat (10) @(posedge clk);
        if (sb.errors == 0) $display("TEST PASSED");
        else                $display("TEST FAILED: %0d errors", sb.errors);
        $finish;
    end

endmodule
