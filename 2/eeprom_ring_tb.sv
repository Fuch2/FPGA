// =============================================================================
// Тестбенч для eeprom_ring и eeprom_store: кольцевой буфер измерений в EEPROM
//
// Два окружения ee_store_env (eeprom_store + i2c_master + имитатор eeprom_model):
//   0 - малое кольцо RECORDS = 8, цикл записи 500 мкс: частые переходы через границу;
//   1 - кольцо как в PROTOCOL.md, RECORDS = BIG_RECORDS = 512 (адреса 0..1023), цикл
//       записи 100 мкс (чтобы симуляция была короче; цикл 5 мс проверяет eeprom_model_tb).
// Эталон - класс RingRef (очередь значений). Окружения идут параллельно.
// Проверки:
//   R1. После сброса POP - res_empty, обращений к шине нет
//   R2. START; 3 записи PUSH, слова лежат в EEPROM по адресам 2n (старший байт первым);
//       POP возвращает их по порядку, потом res_empty; count, wr_ptr, rd_ptr, ovf
//       совпадают с эталоном после каждой команды
//   R3. Указатели переходят через границу кольца при работе без переполнения (записи
//       и чтения вперемежку)
//   R4. Переполнение: ровно RECORDS записей - ovf = 0, count = RECORDS; ещё одна - ovf = 1,
//       самая старая потеряна; RECORDS + 5 записей: POP возвращает записи с пятой,
//       в порядке записи, RECORDS штук; затем res_empty; ovf остаётся до START
//   R5. START сбрасывает указатели, count и ovf; старые записи в EEPROM остаются, но POP
//       их не отдаёт (res_empty)
//   R6. Ошибки EEPROM (present = 0): PUSH и POP дают res_err, указатели и count не меняются;
//       после возврата EEPROM запись не потеряна
//   R7. Сброс платы (rst_n): указатели обнуляются, содержимое EEPROM остаётся, прямое чтение
//       по адресу возвращает записанные значения
//   R8. Прямой доступ через мультиплексор: запись слова и байтов, чтение, запись другого слова
//       через PUSH видно прямым чтением; пока кольцо занято, ring_cmd_ready = 0 и raw_ready = 0
//   R9. Неизвестная команда: res_err, обращений к шине нет
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Окружение: хранилище + мастер + имитатор EEPROM (только для тестбенча)
// -----------------------------------------------------------------------------
module ee_store_env #(
    parameter int RECORDS  = 512,
    parameter int WRITE_US = 100
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        present,
    // кольцо
    input  logic        ring_cmd_valid,
    output logic        ring_cmd_ready,
    input  logic [1:0]  ring_cmd_op,
    input  logic [15:0] ring_cmd_data,
    output logic        ring_done,
    output logic        ring_ok,
    output logic        ring_empty,
    output logic        ring_err,
    output logic [15:0] ring_data,
    output logic        ring_busy,
    output logic        ring_ovf,
    output logic [9:0]  ring_count,
    output logic [8:0]  ring_wr_ptr,
    output logic [8:0]  ring_rd_ptr,
    // прямой доступ
    input  logic        raw_valid,
    output logic        raw_ready,
    input  logic [2:0]  raw_op,
    input  logic [9:0]  raw_addr,
    input  logic [15:0] raw_data,
    input  logic [7:0]  raw_len,
    output logic        raw_rd_valid,
    output logic [7:0]  raw_rd_data,
    output logic        raw_rd_last,
    output logic        raw_done,
    output logic        raw_ok,
    output logic        raw_nack,
    output logic        raw_bus,
    output logic        raw_poll,
    output logic [7:0]  raw_polls,
    output logic        polling,
    // шина
    output logic        ev_start,
    output logic [15:0] model_cycles
);

    localparam int LEN_W  = 8;
    localparam int PTR_W  = $clog2(RECORDS);

    logic             cmd_valid, cmd_ready, cmd_read, cmd_stop;
    logic [6:0]       cmd_addr;
    logic [LEN_W-1:0] cmd_len;
    logic [7:0]       wr_data, rd_data;
    logic             wr_valid, wr_ready, rd_valid, rd_ready, rd_last;
    logic             done, res_nack_addr, res_nack_data, res_timeout, res_arb;
    logic [LEN_W-1:0] res_len;
    logic [PTR_W:0]   count;
    logic [PTR_W-1:0] wr_ptr, rd_ptr;
    logic             model_busy, model_sda_low;
    tri1              scl, sda;

    assign ring_count  = 10'(count);
    assign ring_wr_ptr = 9'(wr_ptr);
    assign ring_rd_ptr = 9'(rd_ptr);

    eeprom_store #(
        .LEN_W      (LEN_W),
        .RECORDS    (RECORDS),
        .DEV_ADDR   (7'h50),
        .PAGE_BYTES (16),
        .POLL_MAX   (100)
    ) u_store (
        .clk (clk), .rst_n (rst_n),
        .ring_cmd_valid (ring_cmd_valid), .ring_cmd_ready (ring_cmd_ready), .ring_cmd_op (ring_cmd_op),
        .ring_cmd_data (ring_cmd_data), .ring_done (ring_done), .ring_ok (ring_ok), .ring_empty (ring_empty),
        .ring_err (ring_err), .ring_data (ring_data), .ring_busy (ring_busy), .ring_ovf (ring_ovf),
        .ring_count (count), .ring_wr_ptr (wr_ptr), .ring_rd_ptr (rd_ptr),
        .raw_valid (raw_valid), .raw_ready (raw_ready), .raw_op (raw_op), .raw_addr (raw_addr),
        .raw_data (raw_data), .raw_len (raw_len),
        .raw_rd_valid (raw_rd_valid), .raw_rd_ready (1'b1), .raw_rd_data (raw_rd_data), .raw_rd_last (raw_rd_last),
        .raw_done (raw_done), .raw_ok (raw_ok), .raw_nack (raw_nack), .raw_bus (raw_bus), .raw_poll (raw_poll),
        .raw_polls (raw_polls), .polling (polling),
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
        .ev_start (ev_start), .ev_stop (), .ev_ack (), .ev_nack (),
        .i2c_scl (scl), .i2c_sda (sda)
    );

    eeprom_model #(
        .CLK_FREQ (50_000_000),
        .ADDR     (7'h50),
        .WRITE_US (WRITE_US)
    ) u_model (
        .clk (clk), .rst_n (rst_n), .present (present), .busy (model_busy), .wr_cycles (model_cycles),
        .scl_in (scl), .sda_in (sda), .sda_low (model_sda_low)
    );
    assign sda = model_sda_low ? 1'b0 : 1'bz;

endmodule

// -----------------------------------------------------------------------------
module eeprom_ring_tb #(
    parameter int BIG_RECORDS = 512                 // размер большого кольца (меньше - быстрее симуляция)
);

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  N_ENV    = 2;

    localparam int CMD_START = 0,
                   CMD_PUSH  = 1,
                   CMD_POP   = 2;

    localparam int OP_WR_BYTE = 0,
                   OP_WR_WORD = 1,
                   OP_RD_BYTE = 2,
                   OP_RD_SEQ  = 3;

    // эталон кольца: очередь значений, указатели
    class RingRef;
        int          records;
        logic [15:0] q [$];
        bit          ovf = 0;
        int          n_push = 0, n_pop = 0, n_drop = 0;      // записей добавлено / прочитано / затёрто
        logic [15:0] slot_val [1024];
        bit          slot_set [1024];

        function new(int n);
            records = n;
            foreach (slot_set[i]) slot_set[i] = 0;
        endfunction

        function void start();
            q.delete();
            ovf = 0;  n_push = 0;  n_pop = 0;  n_drop = 0;
        endfunction

        function void push(logic [15:0] v);
            if (q.size() == records) begin
                void'(q.pop_front());
                ovf = 1;
                n_drop++;
            end
            q.push_back(v);
            slot_val[n_push % records] = v;
            slot_set[n_push % records] = 1;
            n_push++;
        endfunction

        function int count();
            return q.size();
        endfunction

        function int wr_ptr();
            return n_push % records;
        endfunction

        function int rd_ptr();
            return (n_pop + n_drop) % records;
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
    logic [N_ENV-1:0]  present = '1;
    logic [N_ENV-1:0]  ring_cmd_valid = '0, ring_cmd_ready, ring_done, ring_ok, ring_empty, ring_err, ring_busy, ring_ovf;
    logic [1:0]        ring_cmd_op   [N_ENV];
    logic [15:0]       ring_cmd_data [N_ENV];
    logic [15:0]       ring_data     [N_ENV];
    logic [9:0]        ring_count    [N_ENV];
    logic [8:0]        ring_wr_ptr   [N_ENV];
    logic [8:0]        ring_rd_ptr   [N_ENV];
    logic [N_ENV-1:0]  raw_valid = '0, raw_ready, raw_rd_valid, raw_rd_last, raw_done, raw_ok, raw_nack, raw_bus, raw_poll, polling;
    logic [2:0]        raw_op   [N_ENV];
    logic [9:0]        raw_addr [N_ENV];
    logic [15:0]       raw_data [N_ENV];
    logic [7:0]        raw_len  [N_ENV];
    logic [7:0]        raw_rd_data [N_ENV];
    logic [7:0]        raw_polls   [N_ENV];
    logic [N_ENV-1:0]  ev_start;
    logic [15:0]       model_cycles [N_ENV];

    always #(CLK_NS / 2) clk = ~clk;

    ee_store_env #(.RECORDS (8), .WRITE_US (500)) env0 (
        .clk (clk), .rst_n (rst_n), .present (present[0]),
        .ring_cmd_valid (ring_cmd_valid[0]), .ring_cmd_ready (ring_cmd_ready[0]), .ring_cmd_op (ring_cmd_op[0]),
        .ring_cmd_data (ring_cmd_data[0]), .ring_done (ring_done[0]), .ring_ok (ring_ok[0]),
        .ring_empty (ring_empty[0]), .ring_err (ring_err[0]), .ring_data (ring_data[0]),
        .ring_busy (ring_busy[0]), .ring_ovf (ring_ovf[0]), .ring_count (ring_count[0]),
        .ring_wr_ptr (ring_wr_ptr[0]), .ring_rd_ptr (ring_rd_ptr[0]),
        .raw_valid (raw_valid[0]), .raw_ready (raw_ready[0]), .raw_op (raw_op[0]), .raw_addr (raw_addr[0]),
        .raw_data (raw_data[0]), .raw_len (raw_len[0]),
        .raw_rd_valid (raw_rd_valid[0]), .raw_rd_data (raw_rd_data[0]), .raw_rd_last (raw_rd_last[0]),
        .raw_done (raw_done[0]), .raw_ok (raw_ok[0]), .raw_nack (raw_nack[0]), .raw_bus (raw_bus[0]),
        .raw_poll (raw_poll[0]), .raw_polls (raw_polls[0]), .polling (polling[0]),
        .ev_start (ev_start[0]), .model_cycles (model_cycles[0])
    );
    ee_store_env #(.RECORDS (BIG_RECORDS), .WRITE_US (100)) env1 (
        .clk (clk), .rst_n (rst_n), .present (present[1]),
        .ring_cmd_valid (ring_cmd_valid[1]), .ring_cmd_ready (ring_cmd_ready[1]), .ring_cmd_op (ring_cmd_op[1]),
        .ring_cmd_data (ring_cmd_data[1]), .ring_done (ring_done[1]), .ring_ok (ring_ok[1]),
        .ring_empty (ring_empty[1]), .ring_err (ring_err[1]), .ring_data (ring_data[1]),
        .ring_busy (ring_busy[1]), .ring_ovf (ring_ovf[1]), .ring_count (ring_count[1]),
        .ring_wr_ptr (ring_wr_ptr[1]), .ring_rd_ptr (ring_rd_ptr[1]),
        .raw_valid (raw_valid[1]), .raw_ready (raw_ready[1]), .raw_op (raw_op[1]), .raw_addr (raw_addr[1]),
        .raw_data (raw_data[1]), .raw_len (raw_len[1]),
        .raw_rd_valid (raw_rd_valid[1]), .raw_rd_data (raw_rd_data[1]), .raw_rd_last (raw_rd_last[1]),
        .raw_done (raw_done[1]), .raw_ok (raw_ok[1]), .raw_nack (raw_nack[1]), .raw_bus (raw_bus[1]),
        .raw_poll (raw_poll[1]), .raw_polls (raw_polls[1]), .polling (polling[1]),
        .ev_start (ev_start[1]), .model_cycles (model_cycles[1])
    );

    Scoreboard sb = new();
    RingRef    rr [N_ENV];
    int        records [N_ENV];

    // -------------------------------------------------------------------------
    // Мониторы
    // -------------------------------------------------------------------------
    int     done_cnt [N_ENV];
    int     start_cnt [N_ENV];                      // сколько START было на шине
    int     raw_done_cnt [N_ENV];
    logic [N_ENV-1:0] done_q = '0;
    logic [7:0]       rawq [N_ENV][$];

    always @(posedge clk) begin
        for (int i = 0; i < N_ENV; i++) begin
            if (ring_done[i]) begin
                done_cnt[i]++;
                if (done_q[i]) sb.err($sformatf("env %0d: ring_done is wider than 1 clock", i));
                if ((ring_ok[i] + ring_empty[i] + ring_err[i]) != 1)
                    sb.err($sformatf("env %0d: flags ok/empty/err = %b%b%b at ring_done", i,
                                     ring_ok[i], ring_empty[i], ring_err[i]));
            end
            if (raw_done[i]) raw_done_cnt[i]++;
            if (ev_start[i]) start_cnt[i]++;
            if (ring_busy[i] && raw_ready[i])
                sb.err($sformatf("env %0d: raw_ready = 1 while the ring is busy", i));      // в любом такте, и между состояниями кольца
            if (ring_busy[i] && ring_cmd_ready[i])
                sb.err($sformatf("env %0d: ring_cmd_ready = 1 while the ring is busy", i));
            if (raw_rd_valid[i]) rawq[i].push_back(raw_rd_data[i]);
        end
        done_q <= ring_done;
    end

    // -------------------------------------------------------------------------
    // Команда кольца: выполнить и сверить с эталоном
    // -------------------------------------------------------------------------
    bit          g_ok [N_ENV], g_empty [N_ENV], g_err [N_ENV];
    logic [15:0] g_data [N_ENV];

    task automatic ring_op(input int idx, input int op, input logic [15:0] v);
        int d0, guard;
        d0 = done_cnt[idx];
        @(posedge clk);
        ring_cmd_op[idx]    <= 2'(op);
        ring_cmd_data[idx]  <= v;
        ring_cmd_valid[idx] <= 1'b1;
        do @(posedge clk); while (!ring_cmd_ready[idx]);
        ring_cmd_valid[idx] <= 1'b0;
        guard = 0;
        while (done_cnt[idx] == d0 && guard < 5_000_000) begin
            @(posedge clk);
            guard++;
        end
        if (guard >= 5_000_000) sb.err($sformatf("env %0d: command %0d does not finish", idx, op));
        repeat (2) @(posedge clk);
        g_ok[idx]    = ring_ok[idx];
        g_empty[idx] = ring_empty[idx];
        g_err[idx]   = ring_err[idx];
        g_data[idx]  = ring_data[idx];
    endtask

    task automatic check_status(input string name, input int idx);
        if (ring_count[idx] !== 10'(rr[idx].count()))
            sb.err($sformatf("%s env %0d: count = %0d, expected %0d", name, idx, ring_count[idx], rr[idx].count()));
        if (ring_wr_ptr[idx] !== 9'(rr[idx].wr_ptr()))
            sb.err($sformatf("%s env %0d: wr_ptr = %0d, expected %0d", name, idx, ring_wr_ptr[idx], rr[idx].wr_ptr()));
        if (ring_rd_ptr[idx] !== 9'(rr[idx].rd_ptr()))
            sb.err($sformatf("%s env %0d: rd_ptr = %0d, expected %0d", name, idx, ring_rd_ptr[idx], rr[idx].rd_ptr()));
        if (ring_ovf[idx] !== rr[idx].ovf)
            sb.err($sformatf("%s env %0d: ovf = %b, expected %b", name, idx, ring_ovf[idx], rr[idx].ovf));
    endtask

    task automatic do_start(input string name, input int idx);
        int s0;
        s0 = start_cnt[idx];
        ring_op(idx, CMD_START, '0);
        rr[idx].start();
        if (!g_ok[idx]) sb.err($sformatf("%s env %0d: START without ok", name, idx));
        if (start_cnt[idx] != s0) sb.err($sformatf("%s env %0d: START touched the bus", name, idx));
        check_status(name, idx);
    endtask

    task automatic do_push(input string name, input int idx, input logic [15:0] v);
        ring_op(idx, CMD_PUSH, v);
        if (!g_ok[idx] || g_empty[idx] || g_err[idx]) sb.err($sformatf("%s env %0d: PUSH ok/empty/err = %b%b%b", name, idx, g_ok[idx], g_empty[idx], g_err[idx]));
        rr[idx].push(v);
        check_status(name, idx);
    endtask

    task automatic do_pop(input string name, input int idx);
        logic [15:0] exp_v;
        int          s0;
        bit          was_empty;
        s0 = start_cnt[idx];
        was_empty = (rr[idx].count() == 0);
        if (!was_empty) exp_v = rr[idx].q.pop_front();
        ring_op(idx, CMD_POP, '0);
        if (was_empty) begin
            if (!g_empty[idx] || g_ok[idx] || g_err[idx])
                sb.err($sformatf("%s env %0d: POP of an empty ring ok/empty/err = %b%b%b", name, idx, g_ok[idx], g_empty[idx], g_err[idx]));
            if (start_cnt[idx] != s0) sb.err($sformatf("%s env %0d: POP of an empty ring touched the bus", name, idx));
        end else begin
            rr[idx].n_pop++;
            if (!g_ok[idx] || g_empty[idx] || g_err[idx])
                sb.err($sformatf("%s env %0d: POP ok/empty/err = %b%b%b", name, idx, g_ok[idx], g_empty[idx], g_err[idx]));
            else if (g_data[idx] !== exp_v)
                sb.err($sformatf("%s env %0d: POP = %h, expected %h", name, idx, g_data[idx], exp_v));
        end
        check_status(name, idx);
    endtask

    // значение записи с номером n
    function automatic logic [15:0] val(int salt, int n);
        return 16'(16'h1960 + salt * 16'h0100 + n * 16'h0020);
    endfunction

    // слова кольца в памяти модели совпадают с эталоном (старший байт по чётному адресу)
    task automatic check_layout(input string name, input int idx);
        for (int k = 0; k < records[idx]; k++) begin
            logic [7:0] hi, lo;
            if (!rr[idx].slot_set[k]) continue;
            hi = (idx == 0) ? env0.u_model.mem[2*k]     : env1.u_model.mem[2*k];
            lo = (idx == 0) ? env0.u_model.mem[2*k + 1] : env1.u_model.mem[2*k + 1];
            if ({hi, lo} !== rr[idx].slot_val[k])
                sb.err($sformatf("%s env %0d: EEPROM word %0d = %h, expected %h", name, idx, k, {hi, lo}, rr[idx].slot_val[k]));
        end
    endtask

    // -------------------------------------------------------------------------
    // Прямой доступ
    // -------------------------------------------------------------------------
    bit          r_ok [N_ENV], r_nack [N_ENV];

    task automatic raw_op_run(input int idx, input int op, input int addr, input logic [15:0] v, input int len);
        int d0, guard;
        d0 = raw_done_cnt[idx];
        rawq[idx].delete();
        @(posedge clk);
        raw_op[idx]   <= 3'(op);
        raw_addr[idx] <= 10'(addr);
        raw_data[idx] <= v;
        raw_len[idx]  <= 8'(len);
        raw_valid[idx] <= 1'b1;
        do @(posedge clk); while (!raw_ready[idx]);
        raw_valid[idx] <= 1'b0;
        guard = 0;
        while (raw_done_cnt[idx] == d0 && guard < 5_000_000) begin
            @(posedge clk);
            guard++;
        end
        if (guard >= 5_000_000) sb.err($sformatf("env %0d: raw operation %0d does not finish", idx, op));
        repeat (2) @(posedge clk);
        r_ok[idx]   = raw_ok[idx];
        r_nack[idx] = raw_nack[idx];
    endtask

    task automatic check_raw_word(input string name, input int idx, input int addr, input logic [15:0] exp_v);
        raw_op_run(idx, OP_RD_SEQ, addr, '0, 2);
        if (!r_ok[idx] || rawq[idx].size() != 2)
            sb.err($sformatf("%s env %0d: raw read at %0d failed (ok=%b, %0d bytes)", name, idx, addr, r_ok[idx], rawq[idx].size()));
        else if ({rawq[idx][0], rawq[idx][1]} !== exp_v)
            sb.err($sformatf("%s env %0d: raw word at %0d = %h, expected %h", name, idx, addr, {rawq[idx][0], rawq[idx][1]}, exp_v));
    endtask

    // -------------------------------------------------------------------------
    // Сценарий одного окружения
    // -------------------------------------------------------------------------
    task automatic scenario(input int idx);
        int   n, rec, s0;
        rec = records[idx];

        // ---------------------------------------------------------------- R1
        s0 = start_cnt[idx];
        do_pop("R1", idx);

        // ---------------------------------------------------------------- R2
        do_start("R2", idx);
        for (int i = 0; i < 3; i++) do_push("R2 push", idx, val(0, i));
        check_layout("R2", idx);
        check_raw_word("R2 raw", idx, 0, val(0, 0));
        check_raw_word("R2 raw", idx, 4, val(0, 2));
        for (int i = 0; i < 3; i++) do_pop("R2 pop", idx);
        do_pop("R2 empty", idx);

        // ---------------------------------------------------------------- R3
        do_start("R3", idx);
        n = (rec <= 16) ? 3 * rec : 14;
        for (int i = 0; i < n; i++) begin
            do_push("R3 push", idx, val(1, i));
            if (rr[idx].count() > rec / 2) do_pop("R3 pop", idx);       // заполнение не больше половины кольца
        end
        if (rr[idx].ovf) sb.err("R3: reference overflowed");
        while (rr[idx].count() > 0) do_pop("R3 drain", idx);
        do_pop("R3 empty", idx);
        check_layout("R3", idx);

        // ---------------------------------------------------------------- R4
        do_start("R4", idx);
        for (int i = 0; i < rec; i++) do_push("R4 fill", idx, val(2, i));
        if (ring_ovf[idx]) sb.err($sformatf("R4 env %0d: ovf after exactly RECORDS writes", idx));
        if (ring_count[idx] !== 10'(rec)) sb.err($sformatf("R4 env %0d: count = %0d after RECORDS writes", idx, ring_count[idx]));
        do_push("R4 overflow", idx, val(2, rec));
        if (!ring_ovf[idx]) sb.err($sformatf("R4 env %0d: no ovf after RECORDS + 1 writes", idx));
        for (int i = 1; i < 5; i++) do_push("R4 overflow", idx, val(2, rec + i));
        check_layout("R4", idx);
        for (int i = 0; i < rec; i++) do_pop("R4 pop", idx);
        do_pop("R4 empty", idx);
        if (!ring_ovf[idx]) sb.err($sformatf("R4 env %0d: ovf cleared without START", idx));
        if (rr[idx].wr_ptr() != 5) sb.err($sformatf("R4 env %0d: expected wr_ptr 5 after RECORDS + 5 writes", idx));

        // ---------------------------------------------------------------- R5
        do_start("R5", idx);
        do_pop("R5 old data not returned", idx);
        if (rr[idx].count() != 0) sb.err("R5: reference count");
        do_push("R5", idx, val(3, 0));
        do_pop("R5", idx);

        // ---------------------------------------------------------------- R6
        do_start("R6", idx);
        do_push("R6", idx, val(4, 0));
        do_push("R6", idx, val(4, 1));
        present[idx] = 1'b0;
        ring_op(idx, CMD_PUSH, val(4, 2));
        if (!g_err[idx] || g_ok[idx] || g_empty[idx]) sb.err($sformatf("R6 env %0d: PUSH without EEPROM: ok/empty/err = %b%b%b", idx, g_ok[idx], g_empty[idx], g_err[idx]));
        check_status("R6 push error", idx);
        ring_op(idx, CMD_POP, '0);
        if (!g_err[idx] || g_ok[idx] || g_empty[idx]) sb.err($sformatf("R6 env %0d: POP without EEPROM: ok/empty/err = %b%b%b", idx, g_ok[idx], g_empty[idx], g_err[idx]));
        check_status("R6 pop error", idx);
        present[idx] = 1'b1;
        repeat (10) @(posedge clk);
        do_pop("R6 record kept", idx);
        do_pop("R6 record kept", idx);
        do_pop("R6 empty", idx);

        // ---------------------------------------------------------------- R8
        do_start("R8", idx);
        raw_op_run(idx, OP_WR_WORD, 2 * 3, 16'hD00D, 0);
        if (!r_ok[idx]) sb.err("R8: raw word write failed");
        check_raw_word("R8 raw", idx, 6, 16'hD00D);
        raw_op_run(idx, OP_WR_BYTE, 2 * 3 + 1, 16'h00EE, 0);
        check_raw_word("R8 raw byte", idx, 6, 16'hD0EE);
        rr[idx].slot_val[3] = 16'hD0EE;                 // слово 3 изменили прямой записью
        rr[idx].slot_set[3] = 1'b1;
        do_push("R8 push", idx, val(5, 0));
        do_push("R8 push", idx, val(5, 1));
        check_raw_word("R8 raw after push", idx, 2, val(5, 1));
        // пока кольцо занято, прямые запросы не принимаются
        @(posedge clk);
        ring_cmd_op[idx]    <= 2'(CMD_PUSH);
        ring_cmd_data[idx]  <= val(5, 2);
        ring_cmd_valid[idx] <= 1'b1;
        do @(posedge clk); while (!ring_cmd_ready[idx]);
        ring_cmd_valid[idx] <= 1'b0;
        repeat (50) @(posedge clk);
        if (raw_ready[idx]) sb.err($sformatf("R8 env %0d: raw_ready = 1 while the ring works", idx));
        if (ring_cmd_ready[idx]) sb.err($sformatf("R8 env %0d: ring_cmd_ready = 1 while the ring works", idx));
        while (!ring_cmd_ready[idx]) @(posedge clk);
        rr[idx].push(val(5, 2));
        repeat (5) @(posedge clk);
        check_status("R8", idx);
        do_pop("R8", idx);
        do_pop("R8", idx);
        do_pop("R8", idx);
        check_layout("R8", idx);

        // ---------------------------------------------------------------- R9
        s0 = start_cnt[idx];
        ring_op(idx, 3, '0);
        if (!g_err[idx] || g_ok[idx]) sb.err($sformatf("R9 env %0d: unknown command without error", idx));
        if (start_cnt[idx] != s0) sb.err($sformatf("R9 env %0d: unknown command touched the bus", idx));
        check_status("R9", idx);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        logic [15:0] saved [3];
        records[0] = 8;
        records[1] = BIG_RECORDS;
        for (int i = 0; i < N_ENV; i++) begin
            rr[i] = new(records[i]);
            done_cnt[i] = 0;  start_cnt[i] = 0;  raw_done_cnt[i] = 0;
        end
        $timeformat(-6, 2, " us", 0);
        #(CLK_NS * 5);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        fork
            scenario(0);
            scenario(1);
        join

        // ---------------------------------------------------------------- R7
        // сброс платы: указатели обнуляются, содержимое EEPROM остаётся
        for (int idx = 0; idx < N_ENV; idx++) begin
            do_start("R7", idx);
            for (int i = 0; i < 3; i++) do_push("R7 push", idx, val(6, i));
        end
        rst_n = 1'b0;
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);
        for (int idx = 0; idx < N_ENV; idx++) begin
            rr[idx].start();
            check_status("R7 after reset", idx);
        end
        for (int idx = 0; idx < N_ENV; idx++) begin
            for (int i = 0; i < 3; i++) check_raw_word("R7 raw after reset", idx, 2 * i, val(6, i));
            do_pop("R7 old data not returned", idx);
        end

        repeat (10) @(posedge clk);
        if (sb.errors == 0) $display("TEST PASSED");
        else                $display("TEST FAILED: %0d errors", sb.errors);
        $finish;
    end

endmodule
