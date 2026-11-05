// =============================================================================
// ШИМ (PWM) с заданными разрядностью и частотой
//
// Период ШИМ состоит из STEPS = 2^WIDTH - 1 шагов, шаг длится DIV тактов clk:
//   DIV = CLK_FREQ / (PWM_FREQ * STEPS), не меньше 1.
// Выход pwm_out = 1 в первых duty шагах периода:
//   duty = 0            - всегда 0;
//   duty = 2^WIDTH - 1  - всегда 1 (100 %).
// Новое значение duty защёлкивается только в начале периода,
// поэтому смена скважности не даёт укороченных или лишних импульсов.
// period_start - импульс на 1 такт в первом такте каждого периода на выходе.
// После сброса первый период идёт с duty = 0.
// =============================================================================
module pwm #(
    parameter int CLK_FREQ = 50_000_000,  // частота clk, Гц
    parameter int PWM_FREQ = 16_000,      // желаемая частота ШИМ, Гц
    parameter int WIDTH    = 8            // разрядность скважности
) (
    input  logic             clk,
    input  logic             rst_n,         // асинхронный сброс, активный 0
    input  logic [WIDTH-1:0] duty,          // скважность, шагов из STEPS
    output logic             pwm_out,
    output logic             period_start
);

    localparam int STEPS   = 2**WIDTH - 1;
    localparam int DIV_RAW = CLK_FREQ / (PWM_FREQ * STEPS);
    localparam int DIV     = (DIV_RAW < 1) ? 1 : DIV_RAW;
    localparam int DIV_W   = $clog2(DIV + 1);

    logic [DIV_W-1:0] pre_cnt;   // делитель: такты внутри шага
    logic [WIDTH-1:0] step_cnt;  // номер шага в периоде, 0..STEPS-1
    logic [WIDTH-1:0] duty_r;    // скважность текущего периода

    wire step_end   = (pre_cnt == DIV - 1);
    wire period_end = step_end && (step_cnt == STEPS - 1);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pre_cnt      <= '0;
            step_cnt     <= '0;
            duty_r       <= '0;
            pwm_out      <= 1'b0;
            period_start <= 1'b0;
        end else begin
            // счётчики в нуле = первый шаг периода; как и pwm_out, с задержкой 1 такт
            period_start <= (step_cnt == '0) && (pre_cnt == '0);

            if (step_end) begin
                pre_cnt <= '0;
                if (period_end) begin
                    step_cnt <= '0;
                    duty_r   <= duty;          // новая скважность - с начала периода
                end else begin
                    step_cnt <= step_cnt + 1'b1;
                end
            end else begin
                pre_cnt <= pre_cnt + 1'b1;
            end

            // выход отстаёт от счётчиков на 1 такт, длительности не меняются
            pwm_out <= (step_cnt < duty_r);
        end
    end

endmodule
