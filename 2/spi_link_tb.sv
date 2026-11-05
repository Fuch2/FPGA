// =============================================================================
// Тестбенч связки spi_master + spi_slave: линии SPI соединены между собой
//
// Линия MISO общая, слейв отдаёт её в Z при CS_N = 1 (как на плате через вывод
// с подтяжкой). Источник байтов ответа слейва (valid/ready) начинает выдавать
// байты через 4 такта после frame_start и отдаёт заранее заданный список.
// Классы:
//   LinkScoreboard - сравнение: байты, принятые слейвом, = переданные мастером;
//                    байты, принятые мастером, = ответ слейва; события кадров.
// Проверки:
//   1. Кадр протокола A1 00 00 00 с ответом 4 байта (STATUS, D_H, D_L, CHK)
//   2. Несколько кадров подряд с разными данными: между ними CS_N = 1,
//      слейв считает по одному frame_start и frame_end на каждый кадр
//   3. Кадры по 1..8 байт, байты 0x00 / 0xFF / 0x80 / 0x01
//   4. Паузы между байтами (источник мастера не успевает)
//   5. 500 случайных кадров подряд: ни одного искажённого байта,
//      ни одного tx_underrun, ни одного frame_short
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module spi_link_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;

    typedef logic [7:0] bytes_t[$];

    class LinkScoreboard;
        int errors = 0;
        int frames = 0;

        function void err(string msg);
            errors++;
            if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
        endfunction

        function bit same(bytes_t a, bytes_t b);
            if (a.size() != b.size()) return 0;
            foreach (a[i]) if (a[i] !== b[i]) return 0;
            return 1;
        endfunction

        function void check(string name, bytes_t sent, bytes_t slave_got,
                            bytes_t answer, bytes_t master_got,
                            int n_start, int n_end, int n_short, int n_under);
            frames++;
            if (!same(sent, slave_got))
                err($sformatf("%s: slave received %p, master sent %p", name, slave_got, sent));
            if (!same(answer, master_got))
                err($sformatf("%s: master received %p, slave answer %p", name, master_got, answer));
            if (n_start != 1 || n_end != 1)
                err($sformatf("%s: frame_start %0d, frame_end %0d", name, n_start, n_end));
            if (n_short != 0 || n_under != 0)
                err($sformatf("%s: frame_short %0d, tx_underrun %0d", name, n_short, n_under));
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic clk   = 1'b0;
    logic rst_n = 1'b0;

    // мастер
    logic [7:0] m_tx_data  = '0;
    logic       m_tx_valid = 1'b0;
    logic       m_tx_last  = 1'b0;
    logic       m_tx_ready;
    logic [7:0] m_rx_data;
    logic       m_rx_valid;
    logic       m_busy;

    // слейв
    logic [7:0] s_rx_data;
    logic       s_rx_valid;
    logic [7:0] s_tx_data  = '0;
    logic       s_tx_valid = 1'b0;
    logic       s_tx_ready, s_tx_underrun;
    logic       s_frame_start, s_frame_end, s_frame_short;
    logic       s_miso, s_miso_oe;

    // линии SPI
    logic cs_n, sck, mosi;
    tri1  miso_line;
    assign miso_line = s_miso_oe ? s_miso : 1'bz;

    real sck_freq = 1_000_000.0;

    always #(CLK_NS / 2) clk = ~clk;

    spi_master #(
        .CLK_FREQ (CLK_FREQ),
        .SCK_FREQ (1_000_000)
    ) u_master (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_data  (m_tx_data),
        .tx_valid (m_tx_valid),
        .tx_ready (m_tx_ready),
        .tx_last  (m_tx_last),
        .rx_data  (m_rx_data),
        .rx_valid (m_rx_valid),
        .busy     (m_busy),
        .spi_cs_n (cs_n),
        .spi_sck  (sck),
        .spi_mosi (mosi),
        .spi_miso (miso_line)
    );

    spi_slave u_slave (
        .clk         (clk),
        .rst_n       (rst_n),
        .spi_cs_n    (cs_n),
        .spi_sck     (sck),
        .spi_mosi    (mosi),
        .spi_miso    (s_miso),
        .spi_miso_oe (s_miso_oe),
        .rx_data     (s_rx_data),
        .rx_valid    (s_rx_valid),
        .tx_data     (s_tx_data),
        .tx_valid    (s_tx_valid),
        .tx_ready    (s_tx_ready),
        .tx_underrun (s_tx_underrun),
        .frame_start (s_frame_start),
        .frame_end   (s_frame_end),
        .frame_short (s_frame_short)
    );

    LinkScoreboard sb = new();

    // -------------------------------------------------------------------------
    // Источник ответа слейва
    // -------------------------------------------------------------------------
    bytes_t answer;
    int     f_idx, f_wait;

    always @(posedge clk) begin
        if (s_frame_start) begin
            f_idx      = 0;
            f_wait     = 4;
            s_tx_valid <= 1'b0;
        end else if (s_tx_valid && s_tx_ready) begin
            s_tx_valid <= 1'b0;
            f_idx++;
        end else if (!s_tx_valid && f_idx < answer.size()) begin
            if (f_wait > 0) begin
                f_wait--;
            end else begin
                s_tx_data  <= answer[f_idx];
                s_tx_valid <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Журналы
    // -------------------------------------------------------------------------
    bytes_t s_log, m_log;
    int     n_start, n_end, n_short, n_under;

    always @(posedge clk) begin
        if (s_rx_valid)     s_log.push_back(s_rx_data);
        if (m_rx_valid)     m_log.push_back(m_rx_data);
        if (s_frame_start)  n_start++;
        if (s_frame_end)    n_end++;
        if (s_frame_short)  n_short++;
        if (s_tx_underrun)  n_under++;
    end

    // -------------------------------------------------------------------------
    // Передача кадра мастером и проверка
    // -------------------------------------------------------------------------
    task automatic frame(input string name, input bytes_t d, input bytes_t r,
                         input int max_gap = 0);
        s_log.delete();
        m_log.delete();
        n_start = 0; n_end = 0; n_short = 0; n_under = 0;
        answer = r;
        foreach (d[i]) begin
            repeat ($urandom_range(0, max_gap)) @(posedge clk);
            m_tx_data  <= d[i];
            m_tx_valid <= 1'b1;
            m_tx_last  <= (i == d.size() - 1);
            do @(posedge clk); while (!m_tx_ready);
            m_tx_valid <= 1'b0;
            m_tx_data  <= 'x;
            m_tx_last  <= 1'b0;
        end
        repeat (2) @(posedge clk);
        wait (!m_busy);
        repeat (8) @(posedge clk);
        sb.check(name, d, s_log, r, m_log, n_start, n_end, n_short, n_under);
    endtask

    // случайные байты
    function automatic bytes_t rand_bytes(int n);
        bytes_t b;
        b.delete();
        repeat (n) b.push_back($urandom);
        return b;
    endfunction

    function automatic logic [7:0] chk(logic [7:0] s, logic [7:0] h, logic [7:0] l);
        return ~(s ^ h ^ l);
    endfunction

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        $display("[%0t] 1. Protocol frame", $time);
        frame("start", '{8'hA1, 8'h00, 8'h00, 8'h00}, '{8'h81, 8'h12, 8'h34, chk(8'h81, 8'h12, 8'h34)});

        $display("[%0t] 2. Several frames in a row", $time);
        frame("measure", '{8'hA2, 8'h00, 8'h00, 8'h00}, '{8'h83, 8'h00, 8'h01, chk(8'h83, 8'h00, 8'h01)});
        frame("nop",     '{8'h00, 8'h00, 8'h00, 8'h00}, '{8'h83, 8'h00, 8'h02, chk(8'h83, 8'h00, 8'h02)});
        frame("stop",    '{8'hA3, 8'h00, 8'h00, 8'h00}, '{8'h80, 8'h00, 8'h02, chk(8'h80, 8'h00, 8'h02)});

        $display("[%0t] 3. Frames of 1..8 bytes, corner values", $time);
        for (int n = 1; n <= 8; n++)
            frame($sformatf("len%0d", n), rand_bytes(n), rand_bytes(n));
        frame("corner", '{8'h00, 8'hFF, 8'h80, 8'h01}, '{8'hFF, 8'h00, 8'h01, 8'h80});

        $display("[%0t] 4. Gaps between bytes", $time);
        repeat (10) frame("gaps", rand_bytes(4), rand_bytes(4), 60);

        $display("[%0t] 5. 500 random frames back to back", $time);
        for (int k = 0; k < 500; k++)
            frame($sformatf("rand%0d", k), rand_bytes(4), rand_bytes(4));

        $display("------------------------------------------------------------");
        $display("Frames: %0d, ERRORS: %0d", sb.frames, sb.errors);
        if (sb.errors == 0) $display("TEST PASSED");
        else                $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #60ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
