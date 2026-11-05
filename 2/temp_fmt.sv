// =============================================================================
// Преобразование температуры Q8.8 в десятичные цифры для индикатора
//
// Вход (интерфейс valid): in_valid + temp_data - температура Q8.8 со знаком, как
// выдаёт dt_reader: значащие 11 бит D15..D5, шаг 0.125 °C (1 LSB датчика).
// Выход - импульс out_valid на 1 такт через 12 тактов после запроса, затем до следующего
// запроса неизменны:
//   neg  - температура отрицательная;
//   bcd  - модуль температуры в десятых долях градуса, 4 цифры BCD:
//          {сотни, десятки, единицы, десятые}, округление до 0.1 °C (половина - вверх):
//          25.375 °C -> 0254 (25.4), 125 °C -> 1250, -0.125 °C -> neg = 1, 0001;
//          у отрицательных модуль ограничен 99.9 (на индикаторе три разряда и минус).
// Запрос во время преобразования игнорируется. busy = 1, пока идёт преобразование.
//
// Алгоритм: модуль в десятых = (|T| * 5 + 2) / 4, дальше последовательный двоично-
// десятичный перевод «сдвиг и прибавление 3» - 11 тактов на 11 бит, без делителя.
// =============================================================================
module temp_fmt (
    input  logic        clk,
    input  logic        rst_n,          // асинхронный сброс, активный 0

    input  logic        in_valid,
    input  logic [15:0] temp_data,

    output logic        busy,
    output logic        out_valid,
    output logic        neg,
    output logic [15:0] bcd
);

    localparam int BIN_W = 11;              // максимум 1280 десятых (128.0 °C)

    typedef enum logic {IDLE, CONV} state_t;

    state_t                    state;
    logic [BIN_W-1:0]          bin;         // ещё не переведённые биты (старший - следующий)
    logic [15:0]               acc;         // набираемые цифры BCD
    logic [3:0]                cnt;         // сколько битов осталось перевести

    // модуль в шагах 0.125 °C и в десятых долях
    wire signed [10:0] raw     = temp_data[15:5];
    wire               is_neg  = raw[10];
    wire        [10:0] mag     = is_neg ? 11'(-raw) : 11'(raw);       // 0..1024
    wire        [12:0] tenths  = (13'(mag) * 13'd5 + 13'd2) >> 2;     // 0..1280
    wire        [10:0] clamped = (is_neg && tenths > 13'd999) ? 11'd999 : tenths[10:0];

    // «сдвиг и прибавление 3»: цифры 5..9 перед сдвигом увеличиваются на 3
    function automatic logic [15:0] add3(input logic [15:0] v);
        logic [15:0] r;
        for (int i = 0; i < 4; i++)
            r[4*i +: 4] = (v[4*i +: 4] >= 4'd5) ? v[4*i +: 4] + 4'd3 : v[4*i +: 4];
        return r;
    endfunction

    assign busy = (state == CONV);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            bin       <= '0;
            acc       <= '0;
            cnt       <= '0;
            neg       <= 1'b0;
            bcd       <= '0;
            out_valid <= 1'b0;
        end else begin
            out_valid <= 1'b0;

            case (state)
                IDLE: begin
                    if (in_valid) begin
                        bin   <= clamped;
                        acc   <= '0;
                        cnt   <= 4'(BIN_W);
                        neg   <= is_neg;
                        state <= CONV;
                    end
                end

                CONV: begin
                    // следующий бит двоичного числа вдвигается в младшую цифру
                    {acc, bin} <= {add3(acc), bin} << 1;
                    cnt        <= cnt - 1'b1;
                    if (cnt == 4'd1) begin
                        bcd       <= add3(acc) << 1 | 16'(bin[BIN_W-1]);
                        out_valid <= 1'b1;
                        state     <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
