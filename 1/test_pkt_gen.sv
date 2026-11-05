// =============================================================================
// Генератор тестовых данных для пакета МО -> МУ (вместо ДТ и ЦФ)
//
// Пакет номер n (n = 0, 1, 2, ... после restart), все слова Q8.8:
//   meas = MEAS_BASE + n * MEAS_STEP              - «температура» растёт линейно;
//   filt = MEAS_BASE + n * (MEAS_STEP / 2)        - «фильтр» отстаёт вдвое;
//   err  = meas - filt со знаком и с насыщением до -32768..32767.
// Все суммы по модулю 2^16 (как у 16-битного счётчика).
//
// Когда выдаётся пакет:
//   run = 1    - раз в PERIOD_CLKS тактов;
//   single     - импульс: один пакет вне очереди;
//   restart    - импульс: n = 0, первый пакет сразу (если run = 1).
// Если предыдущий пакет ещё не забрали, очередной момент выдачи пропускается.
//
// Выход (valid/ready): meas, filt, err передаются в такте, где valid = 1
//   и ready = 1, и не меняются, пока valid = 1.
// =============================================================================
module test_pkt_gen #(
    parameter int          CLK_FREQ    = 50_000_000,        // частота clk, Гц
    parameter int          PERIOD_CLKS = CLK_FREQ / 10,     // период при run = 1 (100 мс)
    parameter logic [15:0] MEAS_BASE   = 16'h1900,          // 25.0 °C
    parameter logic [15:0] MEAS_STEP   = 16'h0020           // 0.125 °C
) (
    input  logic        clk,
    input  logic        rst_n,      // асинхронный сброс, активный 0

    input  logic        run,
    input  logic        single,
    input  logic        restart,

    output logic [15:0] meas,
    output logic [15:0] filt,
    output logic [15:0] err,
    output logic        valid,
    input  logic        ready
);

    localparam int          TMR_W     = $clog2(PERIOD_CLKS + 1);
    localparam logic [15:0] FILT_STEP = MEAS_STEP >> 1;

    // a - b со знаком, с насыщением
    function automatic logic [15:0] sat_sub(input logic [15:0] a, input logic [15:0] b);
        logic [16:0] d;
        d = {a[15], a} - {b[15], b};
        if (d[16] != d[15]) return d[16] ? 16'h8000 : 16'h7FFF;
        else                return d[15:0];
    endfunction

    logic [TMR_W-1:0] tmr;
    logic [15:0]      next_meas, next_filt;     // значения следующего пакета

    wire tick = run && (tmr == PERIOD_CLKS - 1);
    wire make = (tick || single) && !valid;     // выдать новый пакет

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tmr       <= '0;
            next_meas <= MEAS_BASE;
            next_filt <= MEAS_BASE;
            meas      <= '0;
            filt      <= '0;
            err       <= '0;
            valid     <= 1'b0;
        end else if (restart) begin
            tmr       <= TMR_W'(PERIOD_CLKS - 1);
            next_meas <= MEAS_BASE;
            next_filt <= MEAS_BASE;
            valid     <= 1'b0;
        end else begin
            if (!run || tick) tmr <= '0;
            else              tmr <= tmr + 1'b1;

            if (valid && ready)
                valid <= 1'b0;

            if (make) begin
                meas      <= next_meas;
                filt      <= next_filt;
                err       <= sat_sub(next_meas, next_filt);
                valid     <= 1'b1;
                next_meas <= next_meas + MEAS_STEP;
                next_filt <= next_filt + FILT_STEP;
            end
        end
    end

endmodule
