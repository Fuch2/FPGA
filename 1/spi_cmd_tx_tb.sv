// =============================================================================
// Тестбенч для spi_cmd_tx (конвейер команд МО: Старт, Измерение, Стоп)
//
// Цепочка: spi_cmd_tx -> spi_master -> линии SPI -> spi_slave -> модель МИ.
// Модель МИ работает на уровне байтов: на frame_start защёлкивает слово ответа
// {STATUS, D_H, D_L, CHK} и отдаёт его (valid/ready), на frame_end исполняет
// команду из первого принятого байта кадра. Паузы уменьшены: T_RESP = 40 мкс,
// T_ACK = 20 мкс.
// Классы:
//   MiModel - эталон МИ: флаги RUN / VALID / EMPTY, счётчик значений
//             (D = 0x1000 + номер измерения), слово ответа, исполнение команды.
// Проверки:
//   1. Старт: 2 кадра, MOSI = A1 00 00 00 и 00 00 00 00; ответ на NOP: RUN = 1
//   2. Измерение: ответ на NOP: VALID = 1, D = 0x1000, 0x1001, ...
//   3. Измерение при остановленной МИ: EMPTY = 1, VALID = 0
//   4. Стоп: RUN = 0
//   5. Ответ на первый кадр - прежнее состояние МИ (до команды)
//   6. Пауза между кадрами не короче T_RESP / T_ACK (по линии CS_N)
//   7. Запрос во время выполнения: req_lost, лишних кадров нет
//   8. Одновременные запросы: приоритет Стоп, Старт, Измерение
//   9. Испорченная CHK и «нет МИ» (MISO = 1): rsp_ok = 0
//  10. 100 случайных команд: все ответы совпадают с моделью
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module spi_cmd_tx_tb;

    localparam int  CLK_FREQ  = 50_000_000;
    localparam real CLK_NS    = 1.0e9 / CLK_FREQ;
    localparam int  T_RESP_US = 40;
    localparam int  T_ACK_US  = 20;

    localparam logic [7:0] CMD_START = 8'hA1, CMD_MEASURE = 8'hA2, CMD_STOP = 8'hA3;

    typedef logic [7:0] bytes_t[$];

    class MiModel;
        bit          run   = 0;
        bit          valid = 0;
        bit          empty = 0;
        int          n     = 0;
        logic [15:0] data  = 0;
        bit          corrupt_chk = 0;

        function logic [7:0] status();
            return {1'b1, 2'b00, 1'b0, 1'b0, empty, valid, run};
        endfunction

        function bytes_t word();
            logic [7:0] s = status();
            bytes_t     w;
            w.delete();
            w.push_back(s);
            w.push_back(data[15:8]);
            w.push_back(data[7:0]);
            w.push_back(~(s ^ data[15:8] ^ data[7:0]) ^ (corrupt_chk ? 8'h01 : 8'h00));
            return w;
        endfunction

        function void execute(logic [7:0] cmd);
            case (cmd)
                CMD_START: begin
                    run = 1; valid = 0; empty = 0; n = 0; data = 0;
                end
                CMD_MEASURE: begin
                    valid = 0; empty = 0;
                    if (run) begin
                        data  = 16'h1000 + 16'(n);
                        n++;
                        valid = 1;
                    end else begin
                        empty = 1;
                    end
                end
                CMD_STOP: run = 0;
                default: ;
            endcase
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic clk   = 1'b0;
    logic rst_n = 1'b0;

    logic       req_start = 0, req_measure = 0, req_stop = 0;
    logic       busy, req_lost;
    logic [7:0] m_tx_data, m_rx_data;
    logic       m_tx_valid, m_tx_ready, m_tx_last, m_rx_valid, m_busy;
    logic       rsp_valid, rsp_final, rsp_ok;
    logic [7:0] rsp_cmd, rsp_status;
    logic [15:0] rsp_data;

    logic [7:0] s_rx_data, s_tx_data = '0;
    logic       s_rx_valid, s_tx_valid = 1'b0, s_tx_ready, s_tx_underrun;
    logic       s_frame_start, s_frame_end, s_frame_short;
    logic       s_miso, s_miso_oe;

    logic cs_n, sck, mosi;
    logic no_slave = 1'b0;             // 1 - МИ не подключена, MISO подтянут к 1
    tri1  miso_line;
    assign miso_line = no_slave ? 1'b1 : (s_miso_oe ? s_miso : 1'bz);

    always #(CLK_NS / 2) clk = ~clk;

    spi_cmd_tx #(
        .CLK_FREQ  (CLK_FREQ),
        .T_RESP_US (T_RESP_US),
        .T_ACK_US  (T_ACK_US)
    ) dut (
        .clk (clk), .rst_n (rst_n),
        .req_start (req_start), .req_measure (req_measure), .req_stop (req_stop),
        .busy (busy), .req_lost (req_lost),
        .tx_data (m_tx_data), .tx_valid (m_tx_valid), .tx_ready (m_tx_ready), .tx_last (m_tx_last),
        .rx_data (m_rx_data), .rx_valid (m_rx_valid), .link_busy (m_busy),
        .rsp_valid (rsp_valid), .rsp_final (rsp_final), .rsp_cmd (rsp_cmd),
        .rsp_status (rsp_status), .rsp_data (rsp_data), .rsp_ok (rsp_ok)
    );

    spi_master #(.CLK_FREQ (CLK_FREQ)) u_master (
        .clk (clk), .rst_n (rst_n),
        .tx_data (m_tx_data), .tx_valid (m_tx_valid), .tx_ready (m_tx_ready), .tx_last (m_tx_last),
        .rx_data (m_rx_data), .rx_valid (m_rx_valid), .busy (m_busy),
        .spi_cs_n (cs_n), .spi_sck (sck), .spi_mosi (mosi), .spi_miso (miso_line)
    );

    spi_slave u_slave (
        .clk (clk), .rst_n (rst_n),
        .spi_cs_n (cs_n), .spi_sck (sck), .spi_mosi (mosi),
        .spi_miso (s_miso), .spi_miso_oe (s_miso_oe),
        .rx_data (s_rx_data), .rx_valid (s_rx_valid),
        .tx_data (s_tx_data), .tx_valid (s_tx_valid), .tx_ready (s_tx_ready),
        .tx_underrun (s_tx_underrun),
        .frame_start (s_frame_start), .frame_end (s_frame_end), .frame_short (s_frame_short)
    );

    MiModel mi = new();
    int     errors = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    // -------------------------------------------------------------------------
    // Модель МИ на байтовом интерфейсе слейва
    // -------------------------------------------------------------------------
    bytes_t resp_word;
    bytes_t frame_rx;               // байты текущего кадра
    bytes_t frames_seen[$];         // все принятые кадры по порядку
    bytes_t word_sent[$];           // слова ответа, выданные в этих кадрах
    int     f_idx, f_wait;

    always @(posedge clk) begin
        if (s_frame_start) begin
            resp_word = mi.word();
            f_idx     = 0;
            f_wait    = 3;
            s_tx_valid <= 1'b0;
            frame_rx.delete();
        end else if (s_tx_valid && s_tx_ready) begin
            s_tx_valid <= 1'b0;
            f_idx++;
        end else if (!s_tx_valid && f_idx < resp_word.size()) begin
            if (f_wait > 0) begin
                f_wait--;
            end else begin
                s_tx_data  <= resp_word[f_idx];
                s_tx_valid <= 1'b1;
            end
        end

        if (s_rx_valid) frame_rx.push_back(s_rx_data);

        if (s_frame_end) begin
            frames_seen.push_back(frame_rx);
            word_sent.push_back(resp_word);
            if (frame_rx.size() == 4) mi.execute(frame_rx[0]);
            else                      fail($sformatf("slave frame of %0d bytes", frame_rx.size()));
        end
        if (s_tx_underrun || s_frame_short) fail("tx_underrun / frame_short in slave");
    end

    // -------------------------------------------------------------------------
    // Результаты кадров
    // -------------------------------------------------------------------------
    typedef struct {
        bit          final_f;
        logic [7:0]  cmd, status;
        logic [15:0] data;
        bit          ok;
    } rsp_t;

    rsp_t rsp_log[$];
    int   n_lost = 0;

    always @(posedge clk) begin
        if (rsp_valid)
            rsp_log.push_back('{rsp_final, rsp_cmd, rsp_status, rsp_data, rsp_ok});
        if (req_lost) n_lost++;
    end

    // паузы между кадрами по линии CS_N
    realtime t_cs_rise;
    realtime pause_ns[$];
    int      cs_falls = 0;

    always @(posedge cs_n) t_cs_rise = $realtime;
    always @(negedge cs_n) begin
        cs_falls++;
        if (cs_falls % 2 == 0) pause_ns.push_back($realtime - t_cs_rise);
    end

    // -------------------------------------------------------------------------
    // Запрос команды и проверка результата
    // -------------------------------------------------------------------------
    // ожидание: ответ на первый кадр = слово до команды, на NOP = слово после
    task automatic run_cmd(input string name, input logic [7:0] cmd, input bit expect_ok = 1);
        bytes_t word_before, word_after;
        int     base_rsp    = rsp_log.size();
        int     base_frames = frames_seen.size();
        int     base_pause  = pause_ns.size();
        word_before = mi.word();
        @(posedge clk);
        req_start   <= (cmd == CMD_START);
        req_measure <= (cmd == CMD_MEASURE);
        req_stop    <= (cmd == CMD_STOP);
        @(posedge clk);
        {req_start, req_measure, req_stop} <= '0;
        repeat (3) @(posedge clk);
        wait (!busy && !m_busy);
        repeat (10) @(posedge clk);
        word_after = mi.word();

        if (rsp_log.size() != base_rsp + 2) begin
            fail($sformatf("%s: %0d results, expected 2", name, rsp_log.size() - base_rsp));
            return;
        end
        // кадры на линии
        if (frames_seen.size() != base_frames + 2) begin
            fail($sformatf("%s: %0d frames on the line, expected 2", name, frames_seen.size() - base_frames));
        end else begin
            if (frames_seen[base_frames][0] !== cmd || frames_seen[base_frames][1] !== 8'h00 ||
                frames_seen[base_frames][2] !== 8'h00 || frames_seen[base_frames][3] !== 8'h00)
                fail($sformatf("%s: first frame %p", name, frames_seen[base_frames]));
            if (frames_seen[base_frames + 1][0] !== 8'h00 || frames_seen[base_frames + 1][1] !== 8'h00 ||
                frames_seen[base_frames + 1][2] !== 8'h00 || frames_seen[base_frames + 1][3] !== 8'h00)
                fail($sformatf("%s: second frame %p", name, frames_seen[base_frames + 1]));
        end
        // результаты
        if (rsp_log[base_rsp].final_f || !rsp_log[base_rsp + 1].final_f)
            fail($sformatf("%s: rsp_final flags %b %b", name, rsp_log[base_rsp].final_f, rsp_log[base_rsp + 1].final_f));
        foreach (rsp_log[i]) if (i >= base_rsp) begin
            if (rsp_log[i].cmd !== cmd)
                fail($sformatf("%s: rsp_cmd %h", name, rsp_log[i].cmd));
            if (rsp_log[i].ok !== expect_ok)
                fail($sformatf("%s: rsp_ok = %b, expected %b (result %0d)", name, rsp_log[i].ok, expect_ok, i - base_rsp));
        end
        if (expect_ok) begin
            if ({rsp_log[base_rsp].status, rsp_log[base_rsp].data} !== {word_before[0], word_before[1], word_before[2]})
                fail($sformatf("%s: first reply %h %h, expected %h %h%h", name,
                               rsp_log[base_rsp].status, rsp_log[base_rsp].data, word_before[0], word_before[1], word_before[2]));
            if ({rsp_log[base_rsp + 1].status, rsp_log[base_rsp + 1].data} !== {word_after[0], word_after[1], word_after[2]})
                fail($sformatf("%s: final reply %h %h, expected %h %h%h", name,
                               rsp_log[base_rsp + 1].status, rsp_log[base_rsp + 1].data, word_after[0], word_after[1], word_after[2]));
        end
        // пауза между кадрами
        if (pause_ns.size() == base_pause + 1) begin
            realtime need = (cmd == CMD_MEASURE) ? T_RESP_US * 1000.0 : T_ACK_US * 1000.0;
            if (pause_ns[base_pause] < need)
                fail($sformatf("%s: pause between frames %0.0f ns < %0.0f ns", name, pause_ns[base_pause], need));
        end else begin
            fail($sformatf("%s: pauses measured %0d", name, pause_ns.size() - base_pause));
        end
        $display("[%0t]    %s: status %h -> %h, D = %h", $time, name,
                 rsp_log[base_rsp].status, rsp_log[base_rsp + 1].status, rsp_log[base_rsp + 1].data);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        $display("[%0t] 1. Start", $time);
        run_cmd("start", CMD_START);

        $display("[%0t] 2. Measure x3", $time);
        run_cmd("measure0", CMD_MEASURE);
        run_cmd("measure1", CMD_MEASURE);
        run_cmd("measure2", CMD_MEASURE);

        $display("[%0t] 4. Stop, 3. Measure while stopped", $time);
        run_cmd("stop", CMD_STOP);
        run_cmd("measure_empty", CMD_MEASURE);

        $display("[%0t] 7. Request while busy", $time);
        begin
            automatic int base_frames = frames_seen.size();
            automatic int lost0       = n_lost;
            @(posedge clk);
            req_start <= 1'b1;
            @(posedge clk);
            req_start <= 1'b0;
            repeat (5) @(posedge clk);
            req_measure <= 1'b1;
            @(posedge clk);
            req_measure <= 1'b0;
            repeat (3) @(posedge clk);
            req_stop <= 1'b1;
            @(posedge clk);
            req_stop <= 1'b0;
            repeat (3) @(posedge clk);
            wait (!busy && !m_busy);
            repeat (10) @(posedge clk);
            if (n_lost - lost0 != 2) fail($sformatf("req_lost pulses %0d, expected 2", n_lost - lost0));
            if (frames_seen.size() - base_frames != 2)
                fail($sformatf("%0d frames, expected 2 (only Start)", frames_seen.size() - base_frames));
            if (!mi.run) fail("Start was lost");
        end

        $display("[%0t] 8. Simultaneous requests", $time);
        begin
            automatic int base_frames = frames_seen.size();
            @(posedge clk);
            {req_start, req_measure, req_stop} <= 3'b111;
            @(posedge clk);
            {req_start, req_measure, req_stop} <= '0;
            repeat (3) @(posedge clk);
            wait (!busy && !m_busy);
            repeat (10) @(posedge clk);
            if (frames_seen.size() - base_frames != 2 || frames_seen[base_frames][0] !== CMD_STOP)
                fail("priority: Stop expected");
            @(posedge clk);
            {req_start, req_measure} <= 2'b11;
            @(posedge clk);
            {req_start, req_measure} <= '0;
            repeat (3) @(posedge clk);
            wait (!busy && !m_busy);
            repeat (10) @(posedge clk);
            if (frames_seen[frames_seen.size() - 2][0] !== CMD_START)
                fail("priority: Start over Measure expected");
        end

        $display("[%0t] 9. Corrupted CHK and absent slave", $time);
        mi.corrupt_chk = 1;
        run_cmd("bad_chk", CMD_MEASURE, 0);
        mi.corrupt_chk = 0;
        no_slave = 1'b1;
        run_cmd("no_slave", CMD_MEASURE, 0);
        no_slave = 1'b0;
        run_cmd("recovered", CMD_START);

        $display("[%0t] 10. 100 random commands", $time);
        for (int k = 0; k < 100; k++) begin
            automatic int c = $urandom_range(0, 2);
            run_cmd($sformatf("rand%0d", k), (c == 0) ? CMD_START : (c == 1) ? CMD_MEASURE : CMD_STOP);
        end

        $display("------------------------------------------------------------");
        $display("Results: %0d, ERRORS: %0d", rsp_log.size(), errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #100ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
