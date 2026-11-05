// =============================================================================
// Подавление дребезга кнопки
//
// Асинхронный вход btn синхронизируется двумя триггерами. Новое состояние
// принимается, только если вход держится в нём STABLE_CLKS тактов подряд;
// более короткие импульсы (дребезг, помехи) игнорируются.
//
// Выходы (внутри модуля 1 = нажата, независимо от уровня на выводе):
//   pressed   - состояние кнопки после подавления дребезга;
//   press_p   - импульс на 1 такт в момент нажатия;
//   release_p - импульс на 1 такт в момент отпускания.
// =============================================================================
module debounce #(
    parameter int CLK_FREQ    = 50_000_000,                // частота clk, Гц
    parameter int STABLE_MS   = 10,                        // время устойчивости, мс
    parameter int STABLE_CLKS = CLK_FREQ / 1000 * STABLE_MS,
    parameter bit ACTIVE      = 1'b0                       // уровень нажатой кнопки
) (
    input  logic clk,
    input  logic rst_n,       // асинхронный сброс, активный 0
    input  logic btn,         // вывод кнопки
    output logic pressed,
    output logic press_p,
    output logic release_p
);

    localparam int CNT_W = $clog2(STABLE_CLKS + 1);

    logic [1:0]       sync;   // синхронизатор, хранит уровень вывода
    logic [CNT_W-1:0] cnt;    // сколько тактов вход отличается от pressed

    wire btn_on = (sync[1] == ACTIVE);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync      <= {2{~ACTIVE}};  // кнопка отпущена
            cnt       <= '0;
            pressed   <= 1'b0;
            press_p   <= 1'b0;
            release_p <= 1'b0;
        end else begin
            sync      <= {sync[0], btn};
            press_p   <= 1'b0;
            release_p <= 1'b0;

            if (btn_on == pressed) begin
                cnt <= '0;                     // вход вернулся - отсчёт заново
            end else if (cnt == STABLE_CLKS - 1) begin
                cnt       <= '0;
                pressed   <= btn_on;
                press_p   <=  btn_on;
                release_p <= ~btn_on;
            end else begin
                cnt <= cnt + 1'b1;
            end
        end
    end

endmodule
