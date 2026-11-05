// =============================================================================
// Тестбенч для spi_cmd_parser (команды МО -> МИ: A1 Старт, A2 Измерение, A3 Стоп)
//
// Тестбенч подаёт события слейва напрямую: frame_start, байты rx_data/rx_valid
// со случайными паузами, frame_end (и frame_short для оборванного кадра).
// Классы:
//   ParserModel - эталон: по списку байтов кадра выдаёт ожидаемое событие
//                 (старт / измерение / стоп / NOP / неизвестная / ошибка кадра)
//                 и ожидаемый last_cmd.
// Проверки:
//   1. После сброса: импульсов нет
//   2. Команды A1, A2, A3, 00 (кадр из 4 байт): ровно один соответствующий импульс
//      на 1 такт после frame_end, не раньше
//   3. Остальные 252 значения первого байта: cmd_unknown
//   4. Кадры по 0, 1, 2, 3, 5, 8 байт: frame_err, команда не исполняется
//   5. Оборванный кадр (frame_short) из 4 байт: frame_err
//   6. Последний байт приходит в том же такте, что frame_end: кадр принимается
//   7. Кадры подряд без паузы; байты 2..4 кадра с командой A1..A3 не влияют
//   8. 3000 случайных кадров
//   last_cmd = первый байт последнего кадра правильной длины
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

module spi_cmd_parser_tb;

    localparam real CLK_NS = 20.0;

    localparam int LENS[6] = '{0, 1, 2, 3, 5, 8};

    typedef logic [7:0] bytes_t[$];
    typedef enum int {EV_NONE, EV_START, EV_MEASURE, EV_STOP, EV_NOP, EV_UNKNOWN, EV_ERR} ev_t;

    class ParserModel;
        logic [7:0] last_cmd = '0;
        int         counts[ev_t];

        function new();
            counts[EV_NONE]    = 0;
            counts[EV_START]   = 0;
            counts[EV_MEASURE] = 0;
            counts[EV_STOP]    = 0;
            counts[EV_NOP]     = 0;
            counts[EV_UNKNOWN] = 0;
            counts[EV_ERR]     = 0;
        endfunction

        function ev_t expect_ev(bytes_t b, bit short);
            ev_t e;
            if (short || b.size() != 4) begin
                e = EV_ERR;
            end else begin
                case (b[0])
                    8'hA1:   e = EV_START;
                    8'hA2:   e = EV_MEASURE;
                    8'hA3:   e = EV_STOP;
                    8'h00:   e = EV_NOP;
                    default: e = EV_UNKNOWN;
                endcase
                last_cmd = b[0];
            end
            counts[e]++;
            return e;
        endfunction
    endclass

    // -------------------------------------------------------------------------
    // Сигналы и DUT
    // -------------------------------------------------------------------------
    logic       clk   = 1'b0;
    logic       rst_n = 1'b0;
    logic       frame_start = 0, frame_end = 0, frame_short = 0;
    logic [7:0] rx_data = '0;
    logic       rx_valid = 0;
    logic       cmd_start, cmd_measure, cmd_stop, cmd_nop, cmd_unknown, frame_err;
    logic [7:0] last_cmd;

    always #(CLK_NS / 2) clk = ~clk;

    spi_cmd_parser dut (.*);

    ParserModel model = new();
    int         errors = 0;

    task automatic fail(input string msg);
        errors++;
        if (errors <= 20) $display("[%0t] ERROR: %s", $time, msg);
    endtask

    // монитор выходов: события по тактам
    ev_t seen[$];
    always @(posedge clk) if (rst_n) begin
        automatic int n = cmd_start + cmd_measure + cmd_stop + cmd_nop + cmd_unknown + frame_err;
        if (n > 1) fail("several parser outputs at once");
        if (cmd_start)   seen.push_back(EV_START);
        if (cmd_measure) seen.push_back(EV_MEASURE);
        if (cmd_stop)    seen.push_back(EV_STOP);
        if (cmd_nop)     seen.push_back(EV_NOP);
        if (cmd_unknown) seen.push_back(EV_UNKNOWN);
        if (frame_err)   seen.push_back(EV_ERR);
    end

    // кадр: bytes - принятые байты; last_with_end - последний байт в одном такте с frame_end
    task automatic frame(input string name, input bytes_t b, input bit short = 0,
                         input bit last_with_end = 0, input int max_gap = 3,
                         input int after = 4);
        ev_t exp_ev = model.expect_ev(b, short);
        int  base = seen.size();
        frame_start <= 1'b1;
        @(posedge clk);
        frame_start <= 1'b0;
        foreach (b[i]) begin
            repeat ($urandom_range(0, max_gap)) @(posedge clk);
            rx_data  <= b[i];
            rx_valid <= 1'b1;
            if (last_with_end && i == b.size() - 1) begin
                frame_end   <= 1'b1;
                frame_short <= short;
            end
            @(posedge clk);
            rx_valid <= 1'b0;
            rx_data  <= 'x;
            frame_end   <= 1'b0;
            frame_short <= 1'b0;
        end
        if (!(last_with_end && b.size() != 0)) begin
            repeat ($urandom_range(0, max_gap)) @(posedge clk);
            frame_end   <= 1'b1;
            frame_short <= short;
            @(posedge clk);
            frame_end   <= 1'b0;
            frame_short <= 1'b0;
        end
        repeat (after) @(posedge clk);
        if (seen.size() != base + 1)
            fail($sformatf("%s: %0d events, expected 1 (%s)", name, seen.size() - base, exp_ev.name()));
        else if (seen[base] != exp_ev)
            fail($sformatf("%s: event %s, expected %s", name, seen[base].name(), exp_ev.name()));
        if (last_cmd !== model.last_cmd)
            fail($sformatf("%s: last_cmd = %h, expected %h", name, last_cmd, model.last_cmd));
    endtask

    function automatic bytes_t rand_bytes(int n, logic [7:0] first);
        bytes_t b;
        b.delete();
        for (int i = 0; i < n; i++) b.push_back(i == 0 ? first : 8'($urandom));
        return b;
    endfunction

    // -------------------------------------------------------------------------
    // Сценарий
    // -------------------------------------------------------------------------
    initial begin
        $timeformat(-6, 2, " us", 0);
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        $display("[%0t] 1. Reset state", $time);
        if ({cmd_start, cmd_measure, cmd_stop, cmd_nop, cmd_unknown, frame_err} !== 6'b0)
            fail("outputs after reset");

        $display("[%0t] 2. Commands A1 A2 A3 00", $time);
        frame("start",   '{8'hA1, 8'h00, 8'h00, 8'h00});
        frame("measure", '{8'hA2, 8'h00, 8'h00, 8'h00});
        frame("stop",    '{8'hA3, 8'h00, 8'h00, 8'h00});
        frame("nop",     '{8'h00, 8'h00, 8'h00, 8'h00});

        $display("[%0t] 3. All other first bytes", $time);
        for (int c = 1; c < 256; c++)
            if (c < 8'hA1 || c > 8'hA3) frame($sformatf("cmd%h", c), rand_bytes(4, 8'(c)), 0, 0, 1);

        $display("[%0t] 4. Wrong frame length", $time);
        foreach (LENS[i])
            frame($sformatf("len%0d", LENS[i]), rand_bytes(LENS[i], 8'hA1));

        $display("[%0t] 5. Truncated frame", $time);
        frame("short", rand_bytes(4, 8'hA2), 1);
        frame("short3", rand_bytes(3, 8'hA2), 1);

        $display("[%0t] 6. Last byte together with frame_end", $time);
        frame("with_end_A1", '{8'hA1, 8'h11, 8'h22, 8'h33}, 0, 1);
        frame("with_end_A3", '{8'hA3, 8'h11, 8'h22, 8'h33}, 0, 1);
        frame("with_end_5",  rand_bytes(5, 8'hA2), 0, 1);

        $display("[%0t] 7. Frames back to back, garbage in bytes 2..4", $time);
        repeat (20) frame("garbage", rand_bytes(4, 8'hA1 + 8'($urandom_range(0, 2))), 0, 0, 0, 1);

        $display("[%0t] 8. 3000 random frames", $time);
        for (int k = 0; k < 3000; k++) begin
            automatic int n = ($urandom_range(0, 9) < 7) ? 4 : $urandom_range(0, 7);
            automatic logic [7:0] first = ($urandom_range(0, 1)) ? 8'hA0 + 8'($urandom_range(0, 4)) : 8'($urandom);
            frame($sformatf("rand%0d", k), rand_bytes(n, first), $urandom_range(0, 9) == 0,
                  $urandom_range(0, 3) == 0, 2, 2);
        end

        $display("------------------------------------------------------------");
        $display("start %0d measure %0d stop %0d nop %0d unknown %0d err %0d, ERRORS: %0d",
                 model.counts[EV_START], model.counts[EV_MEASURE], model.counts[EV_STOP],
                 model.counts[EV_NOP], model.counts[EV_UNKNOWN], model.counts[EV_ERR], errors);
        if (errors == 0) $display("TEST PASSED");
        else             $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #20ms;
        $display("TEST FAILED: global timeout");
        $finish;
    end

endmodule
