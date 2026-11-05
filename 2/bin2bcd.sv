// =============================================================================
// Двоичное число -> десятичные цифры (BCD), последовательный «сдвиг с коррекцией +3»
//
// По импульсу start (при busy = 0) число bin запоминается и за IN_W тактов
// переводится в DIGITS десятичных цифр: на каждом такте цифры не меньше 5
// увеличиваются на 3 и всё сдвигается на разряд влево. Результат bcd
// обновляется целиком в такте окончания и держится до следующего перевода,
// поэтому индикатор никогда не видит наполовину посчитанное число.
//   bin   - число, IN_W бит; DIGITS должно хватать на 2**IN_W - 1;
//   busy  - перевод идёт (start игнорируется);
//   bcd   - последнее посчитанное значение, цифра i - bcd[4i+3:4i].
// Для постоянного обновления индикатора достаточно подавать start = !busy.
// =============================================================================
module bin2bcd #(
    parameter int IN_W   = 10,
    parameter int DIGITS = 4
) (
    input  logic                 clk,
    input  logic                 rst_n,         // асинхронный сброс, активный 0

    input  logic                 start,
    input  logic [IN_W-1:0]      bin,
    output logic                 busy,
    output logic [4*DIGITS-1:0]  bcd
);

    localparam int BCD_W = 4 * DIGITS;
    localparam int CNT_W = $clog2(IN_W + 1);

    logic [BCD_W+IN_W-1:0] shreg;       // {цифры, ещё не сдвинутые разряды числа}
    logic [CNT_W-1:0]      iter;        // сколько сдвигов сделано

    // коррекция цифр и сдвиг на разряд влево
    logic [BCD_W-1:0]      digits, digits_adj;
    logic [BCD_W+IN_W-1:0] shreg_next;

    assign digits = shreg[IN_W +: BCD_W];

    always_comb begin
        for (int i = 0; i < DIGITS; i++)
            digits_adj[4*i +: 4] = (digits[4*i +: 4] >= 4'd5) ? digits[4*i +: 4] + 4'd3 : digits[4*i +: 4];
        shreg_next = {digits_adj, shreg[IN_W-1:0]} << 1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shreg <= '0;
            iter  <= '0;
            busy  <= 1'b0;
            bcd   <= '0;
        end else if (!busy) begin
            if (start) begin
                shreg <= {{BCD_W{1'b0}}, bin};
                iter  <= '0;
                busy  <= 1'b1;
            end
        end else begin
            shreg <= shreg_next;
            iter  <= iter + 1'b1;
            if (iter == CNT_W'(IN_W - 1)) begin
                bcd  <= shreg_next[IN_W +: BCD_W];
                busy <= 1'b0;
            end
        end
    end

endmodule
