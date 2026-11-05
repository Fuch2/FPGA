// =============================================================================
// Тестбенч для pwm
//
// Два экземпляра:
//   A - параметры платы: WIDTH = 8, PWM_FREQ = 16 кГц (DIV = 12, период 3060 тактов);
//   B - крайний случай делителя: WIDTH = 4, DIV = 1 (период 15 тактов).
// Для каждого проверяется:
//   1. Длина периода = STEPS * DIV тактов, period_start ровно в начале периода
//   2. Число тактов с pwm_out = 1 за период = duty * DIV
//   3. Импульс один и начинается с первого такта периода
//   4. duty = 0 и duty = max (0 % и 100 %), соседние значения 1 и max-1
//   5. Смена duty в середине периода действует только со следующего периода
//
// Тестбенч самопроверяющийся: в конце выводит TEST PASSED или TEST FAILED.
// =============================================================================
`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// Проверка одного экземпляра pwm: драйвер скважности + монитор выхода
// -----------------------------------------------------------------------------
module pwm_tb_unit #(
    parameter string NAME     = "A",
    parameter int    CLK_FREQ = 50_000_000,
    parameter int    PWM_FREQ = 16_000,
    parameter int    WIDTH    = 8,
    parameter int    PERIODS  = 300          // сколько периодов проверить
) (
    input  logic clk,
    input  logic rst_n,
    output int   errors,
    output logic done
);

    localparam int STEPS   = 2**WIDTH - 1;
    localparam int DIV_RAW = CLK_FREQ / (PWM_FREQ * STEPS);   // как в pwm.sv
    localparam int DIV     = (DIV_RAW < 1) ? 1 : DIV_RAW;
    localparam int PERIOD  = STEPS * DIV;

    logic [WIDTH-1:0] duty = '0;
    logic             pwm_out;
    logic             period_start;

    pwm #(
        .CLK_FREQ (CLK_FREQ),
        .PWM_FREQ (PWM_FREQ),
        .WIDTH    (WIDTH)
    ) dut (.*);

    task automatic fail(input string msg);
        errors++;
        if (errors <= 20) $display("[%0t] %s ERROR: %s", $time, NAME, msg);
    endtask

    // ожидаемая скважность для каждого периода: период 0 после сброса - duty 0,
    // далее - значение, выставленное драйвером в предыдущем периоде
    int exp_q[$];

    // -------------------------------------------------------------------------
    // Драйвер: после начала каждого периода, в его первой половине, меняет duty.
    // Первыми идут крайние значения, затем случайные.
    // -------------------------------------------------------------------------
    initial begin : driver
        int fixed[$];
        int n;
        int d;
        fixed = '{STEPS, 0, 1, STEPS - 1, STEPS / 2, 0, STEPS, STEPS, 1, 0};
        n     = 0;
        exp_q.push_back(0);
        wait (rst_n);
        forever begin
            @(posedge clk iff period_start);
            repeat ($urandom_range(1, PERIOD / 2)) @(posedge clk);
            d = (n < fixed.size()) ? fixed[n] : $urandom_range(0, STEPS);
            n++;
            duty <= d;
            exp_q.push_back(d);
        end
    end

    // -------------------------------------------------------------------------
    // Монитор: считает длину периода и длительность импульса
    // -------------------------------------------------------------------------
    initial begin : monitor
        int  len, high, pulses, periods;
        bit  started, prev;
        int  exp;
        errors  = 0;
        done    = 1'b0;
        started = 1'b0;
        periods = 0;
        wait (rst_n);
        forever begin
            @(posedge clk);
            #1;                                   // значения после фронта
            if (period_start) begin
                if (started) begin
                    exp = exp_q.pop_front();
                    if (len != PERIOD)
                        fail($sformatf("period length %0d, expected %0d", len, PERIOD));
                    if (high != exp * DIV)
                        fail($sformatf("duty %0d: high %0d clk, expected %0d", exp, high, exp * DIV));
                    if (pulses > 1)
                        fail($sformatf("duty %0d: %0d pulses in one period", exp, pulses));
                    periods++;
                    if (periods == PERIODS) begin
                        $display("%s: WIDTH = %0d, DIV = %0d, period = %0d clk, checked %0d periods",
                                 NAME, WIDTH, DIV, PERIOD, periods);
                        done = 1'b1;
                    end
                end
                started = 1'b1;
                len     = 0;
                high    = 0;
                pulses  = 0;
                prev    = 1'b0;
            end
            if (started) begin
                len++;
                if (pwm_out) high++;
                if (pwm_out && !prev) begin
                    pulses++;
                    if (!period_start) fail("pulse does not start at period start");
                end
                prev = pwm_out;
            end
        end
    end

endmodule

// -----------------------------------------------------------------------------
// Верхний уровень тестбенча
// -----------------------------------------------------------------------------
module pwm_tb;

    localparam int  CLK_FREQ = 50_000_000;
    localparam real CLK_NS   = 1.0e9 / CLK_FREQ;

    logic clk   = 1'b0;
    logic rst_n = 1'b0;
    int   err_a, err_b;
    logic done_a, done_b;

    always #(CLK_NS / 2) clk = ~clk;

    // A: параметры платы
    pwm_tb_unit #(.NAME("A"), .CLK_FREQ(CLK_FREQ), .PWM_FREQ(16_000), .WIDTH(8), .PERIODS(300))
        u_a (.clk, .rst_n, .errors(err_a), .done(done_a));

    // B: делитель = 1
    pwm_tb_unit #(.NAME("B"), .CLK_FREQ(CLK_FREQ), .PWM_FREQ(10_000_000), .WIDTH(4), .PERIODS(300))
        u_b (.clk, .rst_n, .errors(err_b), .done(done_b));

    int err_rst = 0;

    initial begin
        // в сбросе выход 0
        repeat (5) @(posedge clk);
        if (u_a.pwm_out !== 1'b0 || u_b.pwm_out !== 1'b0) begin
            err_rst++;
            $display("ERROR: pwm_out != 0 in reset");
        end
        @(negedge clk) rst_n = 1'b1;

        fork
            wait (done_a && done_b);
            begin
                #50ms;
                $display("TEST FAILED: global timeout");
                $finish;
            end
        join_any

        $display("------------------------------------------------------------");
        $display("ERRORS: reset = %0d, A = %0d, B = %0d", err_rst, err_a, err_b);
        if (err_rst == 0 && err_a == 0 && err_b == 0) $display("TEST PASSED");
        else                          $display("TEST FAILED");
        $display("------------------------------------------------------------");
        $finish;
    end

endmodule
