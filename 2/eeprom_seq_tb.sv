// =============================================================================
// Тестбенч для eeprom_seq: сценарии проверки EEPROM (кольцо, прямая запись и чтение)
//
// eeprom_seq + eeprom_store + eeprom_bus с внутренним имитатором eeprom_model (use_model = 1,
// цикл записи 200 мкс), кольцо RECORDS = 8, FILL_EXTRA = 8, CLEAR_WORDS = 8, SEQ_LEN = 4.
// Проверки (после каждого сценария сверяются info_addr, info_data, info_empty, курсор,
// указатели кольца, содержимое памяти имитатора, число импульсов wr_p / rd_p / err_p):
//   S0 START  указатели и курсор в начале; значение записей снова GEN_BASE
//   S4 CLEAR  слова 0..7 стёрты (0xFFFF), 8 записей, info_data = FFFF
//   S1 PUSH   три записи 1960, 1980, 19A0 по адресам 0, 2, 4: информация, память, wr_p,
//             polls_last не меньше 1 (цикл записи 200 мкс)
//   S2 POP    записи по порядку, rd_p; четвёртый POP - info_empty = 1, данные прежние;
//             следующий PUSH снимает info_empty
//   S3 FILL   RECORDS + FILL_EXTRA = 16 записей: ovf = 1, count = 8, wr_p 16 раз; POP
//             возвращает записи с восьмой (1A60)
//   S6 WR_WORD, S5 WR_BYTE, S8 RD_WORD, S7 RD_BYTE, S9 RD_SEQ: данные, курсор (+2, +1, +2, +1, +4),
//             информация о последней операции
//   Ошибка    EEPROM отключена: PUSH и RD_WORD дают error = 1, один импульс err_p, finished
//             приходит, busy снимается; сценарий прерывается; следующий run сбрасывает error
//   run во время выполнения сценария игнорируется
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module eeprom_seq_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  RECORDS  = 8;
    localparam int  ADDR_W   = 10;
    localparam int  LEN_W    = 8;
    localparam int  PTR_W    = $clog2(RECORDS);

    localparam logic [15:0] GEN_BASE = 16'h1960;
    localparam logic [15:0] GEN_STEP = 16'h0020;

    localparam int SC_START = 0, SC_PUSH = 1, SC_POP = 2, SC_FILL = 3, SC_CLEAR = 4,
                   SC_WR_BYTE = 5, SC_WR_WORD = 6, SC_RD_BYTE = 7, SC_RD_WORD = 8, SC_RD_SEQ = 9;

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
    logic                clk = 1'b0, rst_n = 1'b0;
    logic                run = 1'b0;
    logic [3:0]          script = '0;
    logic                busy, finished, error;
    logic [ADDR_W-1:0]   info_addr, cursor;
    logic [15:0]         info_data;
    logic                info_empty;
    logic [7:0]          polls_last;
    logic                wr_p, rd_p, err_p;

    logic                ring_cmd_valid, ring_cmd_ready, ring_done, ring_ok, ring_empty, ring_err, ring_busy, ring_ovf;
    logic [1:0]          ring_cmd_op;
    logic [15:0]         ring_cmd_data, ring_data;
    logic [PTR_W:0]      ring_count;
    logic [PTR_W-1:0]    ring_wr_ptr, ring_rd_ptr;

    logic                raw_valid, raw_ready, raw_rd_valid, raw_rd_ready, raw_rd_last;
    logic                raw_done, raw_ok, raw_nack, raw_bus, raw_poll, polling;
    logic [2:0]          raw_op;
    logic [ADDR_W-1:0]   raw_addr;
    logic [15:0]         raw_data;
    logic [LEN_W-1:0]    raw_len;
    logic [7:0]          raw_rd_data, raw_polls;

    logic                cmd_valid, cmd_ready, cmd_read, cmd_stop;
    logic [6:0]          cmd_addr;
    logic [LEN_W-1:0]    cmd_len;
    logic [7:0]          wr_data, rd_data;
    logic                wr_valid, wr_ready, rd_valid, rd_ready, rd_last;
    logic                done, res_nack_addr, res_nack_data, res_timeout, res_arb;
    logic [LEN_W-1:0]    res_len;
    logic                model_busy;
    logic [15:0]         model_cycles;
    logic                use_model = 1'b1;
    tri1                 scl, sda;

    always #(CLK_NS / 2) clk = ~clk;

    eeprom_seq #(
        .RECORDS (RECORDS), .ADDR_W (ADDR_W), .LEN_W (LEN_W),
        .FILL_EXTRA (8), .CLEAR_WORDS (8), .SEQ_LEN (4), .GEN_BASE (GEN_BASE), .GEN_STEP (GEN_STEP)
    ) dut (
        .clk (clk), .rst_n (rst_n), .run (run), .script (script), .busy (busy), .finished (finished), .error (error),
        .info_addr (info_addr), .info_data (info_data), .info_empty (info_empty), .cursor (cursor),
        .polls_last (polls_last), .wr_p (wr_p), .rd_p (rd_p), .err_p (err_p),
        .ring_cmd_valid (ring_cmd_valid), .ring_cmd_ready (ring_cmd_ready), .ring_cmd_op (ring_cmd_op),
        .ring_cmd_data (ring_cmd_data), .ring_done (ring_done), .ring_ok (ring_ok), .ring_empty (ring_empty),
        .ring_err (ring_err), .ring_data (ring_data), .ring_wr_ptr (ring_wr_ptr), .ring_rd_ptr (ring_rd_ptr),
        .raw_valid (raw_valid), .raw_ready (raw_ready), .raw_op (raw_op), .raw_addr (raw_addr),
        .raw_data (raw_data), .raw_len (raw_len), .raw_rd_valid (raw_rd_valid), .raw_rd_data (raw_rd_data),
        .raw_rd_ready (raw_rd_ready), .raw_done (raw_done), .raw_ok (raw_ok), .raw_polls (raw_polls)
    );

    eeprom_store #(
        .LEN_W (LEN_W), .ADDR_W (ADDR_W), .RECORDS (RECORDS), .DEV_ADDR (7'h50), .PAGE_BYTES (16), .POLL_MAX (100)
    ) store (
        .clk (clk), .rst_n (rst_n),
        .ring_cmd_valid (ring_cmd_valid), .ring_cmd_ready (ring_cmd_ready), .ring_cmd_op (ring_cmd_op),
        .ring_cmd_data (ring_cmd_data), .ring_done (ring_done), .ring_ok (ring_ok), .ring_empty (ring_empty),
        .ring_err (ring_err), .ring_data (ring_data), .ring_busy (ring_busy), .ring_ovf (ring_ovf),
        .ring_count (ring_count), .ring_wr_ptr (ring_wr_ptr), .ring_rd_ptr (ring_rd_ptr),
        .raw_valid (raw_valid), .raw_ready (raw_ready), .raw_op (raw_op), .raw_addr (raw_addr),
        .raw_data (raw_data), .raw_len (raw_len), .raw_rd_valid (raw_rd_valid), .raw_rd_ready (raw_rd_ready),
        .raw_rd_data (raw_rd_data), .raw_rd_last (raw_rd_last), .raw_done (raw_done), .raw_ok (raw_ok),
        .raw_nack (raw_nack), .raw_bus (raw_bus), .raw_poll (raw_poll), .raw_polls (raw_polls), .polling (polling),
        .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_addr (cmd_addr), .cmd_read (cmd_read),
        .cmd_len (cmd_len), .cmd_stop (cmd_stop), .wr_data (wr_data), .wr_valid (wr_valid), .wr_ready (wr_ready),
        .rd_data (rd_data), .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_last (rd_last),
        .done (done), .res_nack_addr (res_nack_addr), .res_nack_data (res_nack_data),
        .res_timeout (res_timeout), .res_arb (res_arb)
    );

    eeprom_bus #(
        .CLK_FREQ (CLK_FREQ), .SCL_FREQ (100_000), .TIMEOUT_US (1_000), .LEN_W (LEN_W),
        .MODEL_ADDR (7'h50), .MODEL_WRITE_US (200)
    ) bus (
        .clk (clk), .rst_n (rst_n), .use_model (use_model), .model_busy (model_busy), .model_cycles (model_cycles),
        .cmd_valid (cmd_valid), .cmd_ready (cmd_ready), .cmd_addr (cmd_addr), .cmd_read (cmd_read),
        .cmd_len (cmd_len), .cmd_stop (cmd_stop), .wr_data (wr_data), .wr_valid (wr_valid), .wr_ready (wr_ready),
        .rd_data (rd_data), .rd_valid (rd_valid), .rd_ready (rd_ready), .rd_last (rd_last),
        .done (done), .res_len (res_len), .res_nack_addr (res_nack_addr), .res_nack_data (res_nack_data),
        .res_timeout (res_timeout), .res_arb (res_arb),
        .ev_start (), .ev_stop (), .ev_ack (), .ev_nack (),
        .i2c_scl (scl), .i2c_sda (sda)
    );

    Scoreboard sb = new();

    // -------------------------------------------------------------------------
    // Сценарий и проверки
    // -------------------------------------------------------------------------
    int n_wr, n_rd, n_err, n_fin;

    always @(posedge clk) begin
        if (wr_p)     n_wr++;
        if (rd_p)     n_rd++;
        if (err_p)    n_err++;
        if (finished) n_fin++;
        if (wr_p && rd_p) sb.err("wr_p and rd_p together");
    end

    // выполнить сценарий; по окончании проверить число импульсов
    task automatic run_script(input int sc, input int exp_wr, input int exp_rd, input int exp_err);
        int guard;
        n_wr = 0;  n_rd = 0;  n_err = 0;  n_fin = 0;
        @(posedge clk);
        script <= 4'(sc);
        run    <= 1'b1;
        @(posedge clk);
        run    <= 1'b0;
        guard = 0;
        while (n_fin == 0 && guard < 20_000_000) begin
            @(posedge clk);
            guard++;
        end
        if (guard >= 20_000_000) sb.err($sformatf("script %0d does not finish", sc));
        repeat (4) @(posedge clk);
        if (busy) sb.err($sformatf("script %0d: busy after finished", sc));
        if (n_fin != 1) sb.err($sformatf("script %0d: %0d finished pulses", sc, n_fin));
        if (n_wr != exp_wr || n_rd != exp_rd || n_err != exp_err)
            sb.err($sformatf("script %0d: wr_p %0d rd_p %0d err_p %0d, expected %0d %0d %0d", sc, n_wr, n_rd, n_err, exp_wr, exp_rd, exp_err));
    endtask

    task automatic check_info(input string name, input int addr, input logic [15:0] data);
        if (info_addr !== ADDR_W'(addr)) sb.err($sformatf("%s: info_addr = %h, expected %h", name, info_addr, addr));
        if (info_data !== data)          sb.err($sformatf("%s: info_data = %h, expected %h", name, info_data, data));
    endtask

    task automatic check_ring(input string name, input int count, input int wr_ptr, input int rd_ptr, input bit ovf);
        if (ring_count !== (PTR_W+1)'(count) || ring_wr_ptr !== PTR_W'(wr_ptr) || ring_rd_ptr !== PTR_W'(rd_ptr) || ring_ovf !== ovf)
            sb.err($sformatf("%s: ring count %0d wr %0d rd %0d ovf %b, expected %0d %0d %0d %b", name, ring_count,
                             ring_wr_ptr, ring_rd_ptr, ring_ovf, count, wr_ptr, rd_ptr, ovf));
    endtask

    task automatic check_word(input string name, input int addr, input logic [15:0] w);
        logic [7:0] hi, lo;
        hi = bus.u_model.mem[addr];
        lo = bus.u_model.mem[addr + 1];
        if ({hi, lo} !== w) sb.err($sformatf("%s: EEPROM word at %0d = %h, expected %h", name, addr, {hi, lo}, w));
    endtask

    function automatic logic [15:0] gen(int n);
        return 16'(GEN_BASE + n * GEN_STEP);
    endfunction

    initial begin
        logic [15:0] word_at_3;
        $timeformat(-6, 2, " us", 0);
        n_wr = 0;  n_rd = 0;  n_err = 0;  n_fin = 0;
        #(CLK_NS * 5);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // ---------------------------------------------------------------- S0
        run_script(SC_START, 0, 0, 0);
        check_ring("S0", 0, 0, 0, 0);
        if (cursor !== '0) sb.err("S0: cursor");
        check_info("S0", 0, 16'h0000);

        // ---------------------------------------------------------------- S4
        run_script(SC_CLEAR, 8, 0, 0);
        for (int w = 0; w < 8; w++) check_word($sformatf("S4 word %0d", w), 2 * w, 16'hFFFF);
        check_info("S4", 14, 16'hFFFF);
        if (polls_last == 0) sb.err("S4: polls_last = 0 after writes with a 200 us cycle");
        if (model_cycles !== 16'd8) sb.err($sformatf("S4: %0d write cycles, expected 8", model_cycles));

        // ---------------------------------------------------------------- S1
        for (int k = 0; k < 3; k++) begin
            run_script(SC_PUSH, 1, 0, 0);
            check_info($sformatf("S1 push %0d", k), 2 * k, gen(k));
            check_ring($sformatf("S1 push %0d", k), k + 1, k + 1, 0, 0);
            check_word($sformatf("S1 push %0d", k), 2 * k, gen(k));
            if (polls_last == 0) sb.err("S1: polls_last = 0");
        end

        // ---------------------------------------------------------------- S2
        for (int k = 0; k < 3; k++) begin
            run_script(SC_POP, 0, 1, 0);
            check_info($sformatf("S2 pop %0d", k), 2 * k, gen(k));
            check_ring($sformatf("S2 pop %0d", k), 2 - k, 3, k + 1, 0);
            if (info_empty) sb.err("S2: info_empty after a successful POP");
        end
        run_script(SC_POP, 0, 1, 0);
        if (!info_empty) sb.err("S2: no info_empty on an empty POP");
        if (info_data !== gen(2)) sb.err("S2: info_data changed on an empty POP");
        check_ring("S2 empty", 0, 3, 3, 0);
        run_script(SC_PUSH, 1, 0, 0);
        if (info_empty) sb.err("S2: PUSH did not clear info_empty");
        check_info("S2 push after empty", 6, gen(3));

        // ---------------------------------------------------------------- S3
        run_script(SC_START, 0, 0, 0);
        run_script(SC_FILL, 16, 0, 0);
        check_ring("S3", 8, 0, 0, 1);
        run_script(SC_POP, 0, 1, 0);
        check_info("S3 pop", 0, gen(8));
        run_script(SC_START, 0, 0, 0);

        // ---------------------------------------------------------------- прямой доступ
        run_script(SC_WR_WORD, 1, 0, 0);                   // по адресу 0, курсор -> 2
        check_info("S6", 0, gen(0));
        check_word("S6", 0, gen(0));
        if (cursor !== ADDR_W'(2)) sb.err($sformatf("S6: cursor = %0d, expected 2", cursor));
        run_script(SC_WR_BYTE, 1, 0, 0);                   // по адресу 2, значение gen(1)[7:0]
        check_info("S5", 2, {8'h00, gen(1)[7:0]});
        if (bus.u_model.mem[2] !== gen(1)[7:0]) sb.err("S5: byte not written");
        if (cursor !== ADDR_W'(3)) sb.err($sformatf("S5: cursor = %0d, expected 3", cursor));
        word_at_3 = {bus.u_model.mem[3], bus.u_model.mem[4]};
        run_script(SC_RD_WORD, 0, 1, 0);
        check_info("S8", 3, word_at_3);
        if (cursor !== ADDR_W'(5)) sb.err($sformatf("S8: cursor = %0d, expected 5", cursor));
        run_script(SC_RD_BYTE, 0, 1, 0);
        check_info("S7", 5, {8'h00, bus.u_model.mem[5]});
        if (cursor !== ADDR_W'(6)) sb.err($sformatf("S7: cursor = %0d, expected 6", cursor));
        run_script(SC_RD_SEQ, 0, 1, 0);
        check_info("S9", 6, {bus.u_model.mem[8], bus.u_model.mem[9]});
        if (cursor !== ADDR_W'(10)) sb.err($sformatf("S9: cursor = %0d, expected 10", cursor));

        // ---------------------------------------------------------------- run во время выполнения
        begin
            int c0;
            c0 = model_cycles;
            @(posedge clk);
            script <= 4'(SC_FILL);  run <= 1'b1;
            @(posedge clk);
            script <= 4'(SC_START);                          // второй run с другим сценарием, пока идёт первый
            repeat (2000) @(posedge clk);
            run <= 1'b0;
            if (!busy) sb.err("run test: scenario did not start");
            n_fin = 0;
            while (n_fin == 0) @(posedge clk);
            if (model_cycles - c0 != 16) sb.err($sformatf("run during a script changed the script: %0d writes, expected 16", model_cycles - c0));
        end

        // ---------------------------------------------------------------- ошибка EEPROM
        run_script(SC_START, 0, 0, 0);
        use_model = 1'b0;                                   // выводы I2C свободны, микросхемы нет
        repeat (10) @(posedge clk);
        run_script(SC_PUSH, 1, 0, 1);
        if (!error) sb.err("error: no error after PUSH without EEPROM");
        check_ring("error push", 0, 0, 0, 0);
        run_script(SC_RD_WORD, 0, 1, 1);
        if (!error) sb.err("error: no error after RD_WORD without EEPROM");
        if (cursor !== '0) sb.err("error: cursor advanced after a failed read");
        run_script(SC_FILL, 1, 0, 1);                       // прерывается на первой же записи
        use_model = 1'b1;
        repeat (10) @(posedge clk);
        run_script(SC_START, 0, 0, 0);
        if (error) sb.err("error: not cleared by the next run");
        run_script(SC_PUSH, 1, 0, 0);
        check_info("after recovery", 0, gen(0));

        repeat (10) @(posedge clk);
        if (sb.errors == 0) $display("TEST PASSED");
        else                $display("TEST FAILED: %0d errors", sb.errors);
        $finish;
    end

endmodule
