// =============================================================================
// Мультиплексор потоков байт на один передатчик UART
//
// N источников с интерфейсом valid/ready и признаком последнего байта last.
// Источник получает передатчик целиком на всю последовательность: после
// первого переданного байта мультиплексор закреплён за ним, пока не уйдёт
// байт с last = 1, поэтому пакеты разных источников не перемешиваются.
// Если передатчик свободен, выбирается источник с меньшим номером
// (0 - высший приоритет).
//
// Входы источника i: in_data[i], in_valid[i], in_last[i]; выход in_ready[i].
// Выход (к UART): байт out_data уходит в такте, где out_valid = 1
//   и out_ready = 1; out_last - признак последнего байта последовательности.
// =============================================================================
module uart_tx_mux #(
    parameter int N = 2                     // число источников
) (
    input  logic            clk,
    input  logic            rst_n,      // асинхронный сброс, активный 0

    input  logic [N-1:0][7:0] in_data,
    input  logic [N-1:0]    in_valid,
    input  logic [N-1:0]    in_last,
    output logic [N-1:0]    in_ready,

    output logic [7:0]      out_data,
    output logic            out_valid,
    input  logic            out_ready,
    output logic            out_last
);

    localparam int SEL_W = (N > 1) ? $clog2(N) : 1;

    logic             locked;   // идёт последовательность источника owner
    logic [SEL_W-1:0] owner;
    logic [SEL_W-1:0] sel;      // источник, подключённый к выходу в этом такте

    always_comb begin
        if (locked) begin
            sel = owner;
        end else begin
            sel = '0;
            for (int i = N - 1; i >= 0; i--)     // последним присвоится меньший номер
                if (in_valid[i]) sel = SEL_W'(i);
        end
    end

    assign out_data  = in_data[sel];
    assign out_valid = in_valid[sel];
    assign out_last  = in_last[sel];
    assign in_ready  = out_ready ? (N'(1) << sel) : '0;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            locked <= 1'b0;
            owner  <= '0;
        end else if (out_valid && out_ready) begin
            locked <= !out_last;
            owner  <= sel;
        end
    end

endmodule
