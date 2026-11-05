// =============================================================================
// Тестбенч для SPI_topA (плата A, МО): кнопки -> команды по SPI -> индикация
//
// К мастеру платы подключён настоящий spi_slave и модель МИ на уровне байтов:
// на frame_start она защёлкивает слово ответа {STATUS, D_H, D_L, CHK}, на frame_end
// исполняет команду (Старт, Измерение, Стоп). D при каждом Измерении 0x1000, 0x1001, ...
// Параметры уменьшены: антидребезг 20 тактов, слот индикатора 20 тактов, вспышка
// 30 тактов, паузы T_RESP = 40 мкс, T_ACK = 20 мкс. Частота SCK настоящая, 1 МГц.
// Классы:
//   MiModel - модель МИ (флаги RUN / VALID / EMPTY, счётчик измерений, слово ответа).
// Проверки:
//   1. После сброса: светодиоды погашены, на линиях покой (CS_N = 1, SCK = 0)
//   2. KEY1 (Старт): по SPI два кадра A1 00 00 00 и 00 00 00 00, модель в RUN,
//      led[2] горит, режим «c XX» показывает A1, «S XX» - STATUS 81
//   3. KEY2 (Измерение) 3 раза: режим «XXXX» показывает 1000, 1001, 1002 (счётчик МИ);
//      STATUS 83; «c XX» показывает A2
//   4. Счётчики: t = число начатых кадров, r = число верных ответов, E = 0
//   5. KEY3 (Стоп): RUN = 0, led[2] погашен; Измерение после Стопа: STATUS 84 (EMPTY),
//      значение D не меняется
//   6. Нажатие во время выполнения команды: потеряно, счётчик ошибок растёт, led[3]
//   7. МИ не подключена (MISO = 1): ответ с неверной CHK, счётчик ошибок, led[3]
//   8. KEY4 переключает режимы по кругу
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module SPI_topA_tb;

    localparam int  CLK_FREQ  = 50_000_000;
    localparam real CLK_NS    = 1.0e9 / CLK_FREQ;
    localparam int  SLOT      = 20;
    localparam int  BLANK     = 2;
    localparam int  FLASH     = 30;
    localparam int  DEB       = 20;
    localparam int  T_RESP_US = 40;
    localparam int  T_ACK_US  = 20;

    localparam logic [7:0] CMD_START = 8'hA1, CMD_MEASURE = 8'hA2, CMD_STOP = 8'hA3;

    // сегменты {g..a} цифр 0..F (как в seg7_dynamic)
    localparam logic [6:0] SEG_REF [16] = '{
        7'h3F, 7'h06, 7'h5B, 7'h4F, 7'h66, 7'h6D, 7'h7D, 7'h07,
        7'h7F, 7'h6F, 7'h77, 7'h7C, 7'h39, 7'h5E, 7'h79, 7'h71
    };

    typedef logic [7:0] bytes_t[$];

    class MiModel;
        bit          run = 0, valid = 0, empty = 0;
        int          n = 0;
        logic [15:0] data = 0;

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
            w.push_back(~(s ^ data[15:8] ^ data[7:0]));
            return w;
        endfunction

        function void execute(logic [7:0] cmd);
            case (cmd)
                CMD_START:   begin run = 1; valid = 0; empty = 0; n = 0; data = 0; end
                CMD_MEASURE: begin
                    valid = 0; empty = 0;
                    if (run) begin data = 16'h1000 + 16'(n); n++; valid = 1; end
                    else     empty = 1;
                end
                CMD_STOP:    run = 0;
                default: ;
            endcase
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk   = 1'b0;
    logic       rst_n = 1'b0;
    logic [3:0] key   = 4'b1111;
    logic       spi_cs_n, spi_sck, spi_mosi;
    logic [7:0] seg;
    logic [3:0] dig, led;

    logic       no_slave = 1'b0;           // 1 - МИ не подключена, MISO подтянут к 1
    logic       s_miso, s_miso_oe;
    tri1        miso_line;
    assign miso_line = no_slave ? 1'b1 : (s_miso_oe ? s_miso : 1'bz);

    always #(CLK_NS / 2) clk = ~clk;

    SPI_topA #(
        .CLK_FREQ   (CLK_FREQ),
        .T_RESP_US  (T_RESP_US),
        .T_ACK_US   (T_ACK_US),
        .DEB_CLKS   (DEB),
        .DIGIT_HZ   (CLK_FREQ / SLOT),
        .BLANK_CLKS (BLANK),
        .FLASH_CLKS (FLASH)
    ) dut (
        .clk (clk), .rst_n (rst_n), .key (key),
        .spi_cs_n (spi_cs_n), .spi_sck (spi_sck), .spi_mosi (spi_mosi), .spi_miso (miso_line),
        .seg (seg), .dig (dig), .led (led)
    );

    // слейв платы B
    logic [7:0] s_rx_data, s_tx_data = '0;
    logic       s_rx_valid, s_tx_valid = 1'b0, s_tx_ready, s_tx_underrun;
    logic       s_frame_start, s_frame_end, s_frame_short;

    spi_slave u_slave (
        .clk (clk), .rst_n (rst_n),
        .spi_cs_n (spi_cs_n), .spi_sck (spi_sck), .spi_mosi (spi_mosi),
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

    wire [7:0] seg_on = ~seg;
    wire [3:0] dig_on = ~dig;
    wire [3:0] led_on = ~led;

    // модель МИ на байтовом интерфейсе слейва
    bytes_t resp_word, frame_rx;
    bytes_t frames_seen[$];
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
            if (frame_rx.size() == 4) mi.execute(frame_rx[0]);
        end
    end

    // -------------------------------------------------------------------------
    // Кнопки и индикатор
    // -------------------------------------------------------------------------
    task automatic press(input int k, input int hold = 3 * DEB);
        key[k] = 1'b0;
        repeat (hold) @(posedge clk);
        key[k] = 1'b1;
        repeat (DEB + 5) @(posedge clk);
    endtask

    // команда целиком выполнена: конвейер свободен, мастер завершил транзакцию
    task automatic wait_done();
        repeat (4) @(posedge clk);
        wait (!dut.u_cmd_tx.busy && !dut.m_busy);
        repeat (10) @(posedge clk);
    endtask

    task automatic cmd_key(input int k);
        press(k);
        wait_done();
    endtask

    string HEX_CHARS = "0123456789ABCDEF";

    // трёхзначное десятичное число, как на индикаторе
    function automatic string dec3(input int n);
        return $sformatf("%0d%0d%0d", (n / 100) % 10, (n / 10) % 10, n % 10);
    endfunction

    // картинка индикатора -> строка из 4 символов (dig3..dig0); пустой разряд - пробел
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
        logic [3:0] lit;
        lit = '0;
        foreach (f[i]) f[i] = '0;
        repeat (2 * 4 * SLOT) begin
            @(posedge clk);
            for (int i = 0; i < 4; i++)
                if (dig_on[i]) begin
                    lit[i] = 1'b1;
                    f[i]   = seg_on;
                end
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

    // KEY4 до нужного режима
    task automatic show_mode(input int target);
        string t;
        repeat (7) begin
            read_text(t);
            if (mode_of(t) == target) return;
            press(3);
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
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        $display("[%0t] 1. Reset state", $time);
        if (led_on !== 4'b0000) fail($sformatf("led = %b after reset", led_on));
        if (spi_cs_n !== 1'b1 || spi_sck !== 1'b0) fail("SPI lines are not idle after reset");

        $display("[%0t] 2. KEY1 - Start", $time);
        cmd_key(0);
        n_tx += 2; n_rx += 2;
        if (frames_seen.size() != 2) begin
            fail($sformatf("%0d frames on SPI, expected 2", frames_seen.size()));
        end else begin
            if (frames_seen[0][0] !== CMD_START) fail($sformatf("first frame %p", frames_seen[0]));
            if (frames_seen[1][0] !== 8'h00)     fail($sformatf("second frame %p", frames_seen[1]));
        end
        if (!mi.run) fail("MI model is not running after Start");
        repeat (FLASH + 10) @(posedge clk);
        if (led_on[2] !== 1'b1) fail("led[2] (RUN) is off after Start");
        expect_text("cmd after Start", 1, "c A1");
        expect_text("status after Start", 2, "5 81");

        $display("[%0t] 3. KEY2 - Measure x3", $time);
        for (int k = 0; k < 3; k++) begin
            cmd_key(1);
            n_tx += 2; n_rx += 2;
            repeat (FLASH + 10) @(posedge clk);
            expect_text($sformatf("data after Measure %0d", k), 0, $sformatf("%h", 16'(16'h1000 + k)));
        end
        expect_text("cmd after Measure", 1, "c A2");
        expect_text("status after Measure", 2, "5 83");

        $display("[%0t] 4. Counters", $time);
        expect_text("tx frames", 3, {"t", dec3(n_tx)});
        expect_text("rx frames", 4, {"r", dec3(n_rx)});
        expect_text("errors",    5, "E000");
        if (led_on[3] !== 1'b0) fail("led[3] is on without errors");

        $display("[%0t] 5. KEY3 - Stop, Measure when stopped", $time);
        cmd_key(2);
        n_tx += 2; n_rx += 2;
        repeat (FLASH + 10) @(posedge clk);
        if (mi.run) fail("MI model is still running after Stop");
        if (led_on[2] !== 1'b0) fail("led[2] (RUN) is on after Stop");
        cmd_key(1);
        n_tx += 2; n_rx += 2;
        expect_text("status EMPTY", 2, "5 84");
        expect_text("data unchanged", 0, "1002");

        $display("[%0t] 6. Key during a command", $time);
        press(0);                       // Старт
        press(1);                       // Измерение, пока Старт ещё выполняется: потеряно
        wait_done();
        n_tx += 2; n_rx += 2; n_err += 1;
        if (frames_seen.size() != 14)
            fail($sformatf("%0d frames on SPI, expected 14 (lost key must not send a frame)", frames_seen.size()));
        repeat (FLASH + 10) @(posedge clk);
        if (led_on[3] !== 1'b1) fail("led[3] is off after a lost request");
        expect_text("errors after lost key", 5, {"E", dec3(n_err)});

        $display("[%0t] 7. No slave connected (MISO = 1)", $time);
        no_slave = 1'b1;
        cmd_key(1);
        n_tx += 2; n_err += 2;
        no_slave = 1'b0;
        expect_text("errors without MI", 5, {"E", dec3(n_err)});
        expect_text("tx frames", 3, {"t", dec3(n_tx)});
        expect_text("rx frames", 4, {"r", dec3(n_rx)});

        $display("[%0t] 8. KEY4 cycles through 6 modes", $time);
        begin
            string first, now;
            read_text(first);
            repeat (6) press(3);
            read_text(now);
            if (now != first) fail($sformatf("modes did not wrap: \"%s\" -> \"%s\"", first, now));
        end

        $display("------------------------------------------------------------");
        $display("Frames on SPI: %0d, ERRORS: %0d", frames_seen.size(), errors);
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
