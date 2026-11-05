// =============================================================================
// Очередь (FIFO) слов на DEPTH элементов, интерфейс valid/ready с обеих сторон
//
// Вход:  in_data принимается в такте, где in_valid = 1 и in_ready = 1;
//        in_ready = 0, когда очередь заполнена.
// Выход: out_data - самое старое слово, out_valid = 1, пока очередь не пуста;
//        слово уходит в такте, где out_valid = 1 и out_ready = 1.
// Запись и чтение в одном такте допускаются (в том числе при полной очереди
// запись не принимается - in_ready от полноты, без обхода).
// count - число слов в очереди (для отладки и индикации).
//
// В UART_top2 стоит перед эхом пакетов: если ПК передаёт чуть быстрее, чем
// отвечает ПЛИС, очередь сглаживает отставание на пачке пакетов.
// =============================================================================
module pkt_fifo #(
    parameter int WIDTH = 48,               // ширина слова, бит
    parameter int DEPTH = 4                 // число слов, степень двойки
) (
    input  logic             clk,
    input  logic             rst_n,     // асинхронный сброс, активный 0

    input  logic [WIDTH-1:0] in_data,
    input  logic             in_valid,
    output logic             in_ready,

    output logic [WIDTH-1:0] out_data,
    output logic             out_valid,
    input  logic             out_ready,

    output logic [$clog2(DEPTH):0] count
);

    localparam int AW = $clog2(DEPTH);

    logic [WIDTH-1:0] mem [DEPTH];
    logic [AW:0]      wr_ptr, rd_ptr;        // старший бит - признак круга

    wire empty = (wr_ptr == rd_ptr);
    wire full  = (wr_ptr[AW] != rd_ptr[AW]) && (wr_ptr[AW-1:0] == rd_ptr[AW-1:0]);
    wire wr    = in_valid && in_ready;
    wire rd    = out_valid && out_ready;

    assign in_ready  = !full;
    assign out_valid = !empty;
    assign out_data  = mem[rd_ptr[AW-1:0]];
    assign count     = wr_ptr - rd_ptr;

    always_ff @(posedge clk) begin
        if (wr)
            mem[wr_ptr[AW-1:0]] <= in_data;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
        end else begin
            if (wr) wr_ptr <= wr_ptr + 1'b1;
            if (rd) rd_ptr <= rd_ptr + 1'b1;
        end
    end

endmodule
