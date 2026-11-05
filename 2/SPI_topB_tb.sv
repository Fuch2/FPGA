// =============================================================================
// Тестбенч для SPI_topB (плата B, МИ): кадры от мастера -> заглушка МИ -> ответ, индикация
//
// К плате подключён настоящий spi_master (копия из платы A), тестбенч играет роль
// МО: передаёт кадры {CMD, 00, 00, 00} и читает ответ {STATUS, D_H, D_L, CHK}.
// Измерение - как в протоколе: кадр A2, пауза, кадр NOP с ответом.
// Параметры уменьшены: время «чтения EEPROM» заглушкой 40 мкс, антидребезг
// 20 тактов, слот индикатора 20 тактов, вспышка 30 тактов. SCK настоящий, 1 МГц.
// Классы:
//   Reply - разбор ответа: STATUS, D, проверка CHK = ~(STATUS ^ D_H ^ D_L), MARK.
// Проверки:
//   1. После сброса: светодиоды погашены; MISO отключён (Z) при CS_N = 1
//   2. Старт, затем NOP: STATUS 0x81 (RUN), CHK верна; led[2] горит
//   3. Измерение, NOP сразу (до конца «чтения»): BUSY = 1, VALID = 0
//   4. Измерение x3, NOP после паузы: STATUS 0x83, D = 1, 2, 3
//   5. Ответ на сам кадр с командой - прежнее состояние (слово защёлкнуто на спаде CS_N)
//   6. Стоп: RUN = 0, led[2] погашен; Измерение после Стопа: EMPTY (STATUS 0x86 -> 0x84)
//   7. Индикатор: «0003» (D), «c A3» (последняя команда), «S XX» (STATUS), счётчики t / r
//   8. Неизвестная команда 0x5A и кадр из 3 байт: ошибки, команда не исполняется, led[3]
//   9. KEY2 обнуляет счётчики и гасит led[3]; KEY1 переключает режимы по кругу
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module SPI_topB_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;
    localparam int  SLOT     = 20;
    localparam int  BLANK    = 2;
    localparam int  FLASH    = 30;
    localparam int  DEB      = 20;
    localparam int  MEAS_US  = 40;
    localparam int  PAUSE_US = 100;         // пауза МО между кадрами

    localparam logic [7:0] CMD_START = 8'hA1, CMD_MEASURE = 8'hA2, CMD_STOP = 8'hA3, CMD_NOP = 8'h00;

    // сегменты {g..a} цифр 0..F (как в seg7_dynamic)
    localparam logic [6:0] SEG_REF [16] = '{
        7'h3F, 7'h06, 7'h5B, 7'h4F, 7'h66, 7'h6D, 7'h7D, 7'h07,
        7'h7F, 7'h6F, 7'h77, 7'h7C, 7'h39, 7'h5E, 7'h79, 7'h71
    };

    typedef logic [7:0] bytes_t[$];

    class Reply;
        logic [7:0]  status;
        logic [15:0] data;
        logic [7:0]  chk;

        function new(bytes_t b);
            status = b[0];
            data   = {b[1], b[2]};
            chk    = b[3];
        endfunction

        function bit chk_ok();
            return chk === ~(status ^ data[15:8] ^ data[7:0]);
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk   = 1'b0;
    logic       rst_n = 1'b0;
    logic [3:0] key   = 4'b1111;
    logic       cs_n, sck, mosi;
    tri1        miso_line;
    wire        miso_pin;               // вывод MISO платы B как есть (Z при CS_N = 1)
    logic [7:0] seg;
    logic [3:0] dig, led;

    // мастер
    logic [7:0] m_tx_data = '0, m_rx_data;
    logic       m_tx_valid = 1'b0, m_tx_last = 1'b0, m_tx_ready, m_rx_valid, m_busy;

    always #(CLK_NS / 2) clk = ~clk;

    assign miso_line = miso_pin;

    SPI_topB #(
        .CLK_FREQ   (CLK_FREQ),
        .MEAS_US    (MEAS_US),
        .DEB_CLKS   (DEB),
        .DIGIT_HZ   (CLK_FREQ / SLOT),
        .BLANK_CLKS (BLANK),
        .FLASH_CLKS (FLASH)
    ) dut (
        .clk (clk), .rst_n (rst_n), .key (key),
        .spi_cs_n (cs_n), .spi_sck (sck), .spi_mosi (mosi), .spi_miso (miso_pin),
        .seg (seg), .dig (dig), .led (led)
    );

    spi_master #(.CLK_FREQ (CLK_FREQ)) u_master (
        .clk (clk), .rst_n (rst_n),
        .tx_data (m_tx_data), .tx_valid (m_tx_valid), .tx_ready (m_tx_ready), .tx_last (m_tx_last),
        .rx_data (m_rx_data), .rx_valid (m_rx_valid), .busy (m_busy),
        .spi_cs_n (cs_n), .spi_sck (sck), .spi_mosi (mosi), .spi_miso (miso_line)
    );

    int errors = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    wire [7:0] seg_on = ~seg;
    wire [3:0] dig_on = ~dig;
    wire [3:0] led_on = ~led;

    // MISO в Z при CS_N = 1 (с задержкой синхронизатора)
    int cs_high_clks = 0;
    always @(posedge clk) begin
        cs_high_clks <= cs_n ? cs_high_clks + 1 : 0;
        if (rst_n && cs_n && cs_high_clks > 5 && miso_pin !== 1'bz)
            fail("MISO is not released (Z) while CS_N = 1");
    end

    // -------------------------------------------------------------------------
    // Роль МО: кадр по SPI
    // -------------------------------------------------------------------------
    bytes_t rx_got;
    always @(posedge clk) if (m_rx_valid) rx_got.push_back(m_rx_data);

    task automatic send_frame(input bytes_t d, output bytes_t r);
        @(posedge clk);                 // начало кадра - строго после фронта, иначе гонка с мастером
        rx_got.delete();
        foreach (d[i]) begin
            m_tx_data  <= d[i];
            m_tx_valid <= 1'b1;
            m_tx_last  <= (i == d.size() - 1);
            do @(posedge clk); while (!m_tx_ready);
            m_tx_valid <= 1'b0;
            m_tx_last  <= 1'b0;
        end
        repeat (2) @(posedge clk);
        wait (!m_busy);
        repeat (8) @(posedge clk);
        r = rx_got;
    endtask

    // команда и чтение ответа: кадр с командой, пауза, кадр NOP; answer - ответ на NOP,
    // first - ответ на кадр с командой
    task automatic command(input logic [7:0] cmd, input int pause_us, output Reply first, output Reply answer);
        bytes_t r1, r2;
        send_frame('{cmd, 8'h00, 8'h00, 8'h00}, r1);
        #(pause_us * 1000.0);
        send_frame('{CMD_NOP, 8'h00, 8'h00, 8'h00}, r2);
        first  = new(r1);
        answer = new(r2);
        if (!first.chk_ok())  fail($sformatf("cmd %h: CHK of the first reply is wrong: %p", cmd, r1));
        if (!answer.chk_ok()) fail($sformatf("cmd %h: CHK of the answer is wrong: %p", cmd, r2));
        if (!first.status[7] || !answer.status[7]) fail($sformatf("cmd %h: MARK = 0", cmd));
    endtask

    task automatic expect_reply(input string name, Reply a, input logic [7:0] status, input logic [15:0] data);
        if (a.status !== status || a.data !== data)
            fail($sformatf("%s: STATUS %h D %h, expected %h %h", name, a.status, a.data, status, data));
        else
            $display("[%0t]    %s: STATUS %h D %h", $time, name, a.status, a.data);
    endtask

    // -------------------------------------------------------------------------
    // Кнопки и индикатор
    // -------------------------------------------------------------------------
    task automatic press(input int k, input int hold = 3 * DEB);
        key[k] = 1'b0;
        repeat (hold) @(posedge clk);
        key[k] = 1'b1;
        repeat (DEB + 5) @(posedge clk);
    endtask

    string HEX_CHARS = "0123456789ABCDEF";

    function automatic string dec3(input int n);
        return $sformatf("%0d%0d%0d", (n / 100) % 10, (n / 10) % 10, n % 10);
    endfunction

    function automatic string seg_char(logic [7:0] f);
        if (f[6:0] === 7'h00) return " ";
        for (int i = 0; i < 16; i++)
            if (f[6:0] === SEG_REF[i]) return HEX_CHARS.substr(i, i);
        case (f[6:0])
            7'h58:   return "c";
            7'h78:   return "t";
            7'h50:   return "r";
            default: return "?";
        endcase
    endfunction

    task automatic read_text(output string text);
        logic [7:0] f[4];
        foreach (f[i]) f[i] = '0;
        repeat (2 * 4 * SLOT) begin
            @(posedge clk);
            for (int i = 0; i < 4; i++)
                if (dig_on[i]) f[i] = seg_on;
        end
        text = {seg_char(f[3]), seg_char(f[2]), seg_char(f[1]), seg_char(f[0])};
    endtask

    // номер режима по картинке: 0 - D (4 цифры), 1 - c, 2 - S (выглядит как 5), 3 - t, 4 - r, 5 - E
    function automatic int mode_of(string t);
        if (t[0] == "c")                  return 1;
        if (t[0] == "5" && t[1] == " ")   return 2;
        if (t[0] == "t")                  return 3;
        if (t[0] == "r")                  return 4;
        if (t[0] == "E" && t[1] == "0")   return 5;
        return 0;
    endfunction

    task automatic show_mode(input int target);
        string t;
        repeat (7) begin
            read_text(t);
            if (mode_of(t) == target) return;
            press(0);                       // KEY1 - следующий режим
        end
        fail($sformatf("mode %0d is not reachable, display \"%s\"", target, t));
    endtask

    task automatic expect_text(input string name, input int target, input string want);
        string t;
        show_mode(target);
        read_text(t);
        if (t != want) fail($sformatf("%s: display \"%s\", expected \"%s\"", name, t, want));
        else           $display("[%0t]    %s: \"%s\"", $time, name, t);
    endtask

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    int n_tx = 0, n_rx = 0, n_err = 0;      // ожидаемые счётчики t, r, E

    initial begin
        Reply first, answer;
        bytes_t raw;
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        $display("[%0t] 1. Reset state", $time);
        if (led_on !== 4'b0000) fail($sformatf("led = %b after reset", led_on));

        $display("[%0t] 2. Start", $time);
        command(CMD_START, PAUSE_US, first, answer);
        n_tx += 2; n_rx += 2;
        expect_reply("first reply (before Start)", first, 8'h80, 16'h0000);
        expect_reply("answer to Start", answer, 8'h81, 16'h0000);
        repeat (FLASH + 10) @(posedge clk);
        if (led_on[2] !== 1'b1) fail("led[2] (RUN) is off after Start");

        $display("[%0t] 3. Measure, NOP before the end of reading", $time);
        command(CMD_MEASURE, 5, first, answer);
        n_tx += 2; n_rx += 2;
        expect_reply("answer while BUSY", answer, 8'h89, 16'h0000);
        #(MEAS_US * 1000.0);
        send_frame('{CMD_NOP, 8'h00, 8'h00, 8'h00}, raw);
        n_tx += 1; n_rx += 1;
        answer = new(raw);
        expect_reply("answer after BUSY", answer, 8'h83, 16'h0001);

        $display("[%0t] 4. Measure x2 more", $time);
        for (int k = 2; k <= 3; k++) begin
            command(CMD_MEASURE, PAUSE_US, first, answer);
            n_tx += 2; n_rx += 2;
            expect_reply($sformatf("first reply #%0d", k), first, 8'h83, 16'(k - 1));
            expect_reply($sformatf("answer #%0d", k), answer, 8'h83, 16'(k));
        end

        $display("[%0t] 7. Indication", $time);
        repeat (FLASH + 10) @(posedge clk);
        expect_text("data", 0, "0003");
        expect_text("cmd", 1, "c A2");
        expect_text("status", 2, "5 83");
        expect_text("tx frames", 3, {"t", dec3(n_tx)});
        expect_text("rx frames", 4, {"r", dec3(n_rx)});
        expect_text("errors", 5, "E000");

        $display("[%0t] 6. Stop, Measure when stopped", $time);
        command(CMD_STOP, PAUSE_US, first, answer);
        n_tx += 2; n_rx += 2;
        expect_reply("answer to Stop", answer, 8'h82, 16'h0003);
        repeat (FLASH + 10) @(posedge clk);
        if (led_on[2] !== 1'b0) fail("led[2] (RUN) is on after Stop");
        command(CMD_MEASURE, PAUSE_US, first, answer);
        n_tx += 2; n_rx += 2;
        expect_reply("answer when stopped", answer, 8'h84, 16'h0003);
        expect_text("cmd after Stop", 1, "c A2");

        $display("[%0t] 8. Unknown command and a 3-byte frame", $time);
        send_frame('{8'h5A, 8'h00, 8'h00, 8'h00}, raw);
        n_tx += 1; n_err += 1;
        send_frame('{CMD_START, 8'h00, 8'h00}, raw);
        n_tx += 1; n_err += 1;
        send_frame('{CMD_NOP, 8'h00, 8'h00, 8'h00}, raw);
        n_tx += 1; n_rx += 1;
        answer = new(raw);
        expect_reply("Start in a 3-byte frame was ignored", answer, 8'h84, 16'h0003);
        repeat (FLASH + 10) @(posedge clk);
        if (led_on[3] !== 1'b1) fail("led[3] is off after errors");
        expect_text("errors", 5, {"E", dec3(n_err)});
        expect_text("tx frames", 3, {"t", dec3(n_tx)});
        expect_text("rx frames", 4, {"r", dec3(n_rx)});

        $display("[%0t] 9. KEY2 clear, KEY1 mode cycle", $time);
        press(1);
        repeat (5) @(posedge clk);
        if (led_on[3] !== 1'b0) fail("led[3] is still on after KEY2");
        expect_text("tx frames after clear", 3, "t000");
        expect_text("errors after clear", 5, "E000");
        begin
            string t0, t1;
            read_text(t0);
            repeat (6) press(0);
            read_text(t1);
            if (t0 != t1) fail($sformatf("modes did not wrap: \"%s\" -> \"%s\"", t0, t1));
        end

        $display("------------------------------------------------------------");
        $display("ERRORS: %0d", errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #30ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
