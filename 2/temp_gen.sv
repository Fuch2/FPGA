// =============================================================================
// Генератор последовательности температур для имитатора ДТ
//
// Температура - знаковое 11-битное число в шагах датчика LM75A: 1 LSB = 0.125 °C,
// диапазон -128 .. +127.875 °C (значение 25.375 °C = 203). Все параметры
// (gen_lo, gen_hi, gen_step) заданы в тех же единицах.
//
// По импульсу tick выдаётся следующее значение последовательности:
//   MODE_CONST  (0) - всегда gen_lo;
//   MODE_LINEAR (1) - линейный рост: +gen_step за тик от gen_lo до gen_hi, затем стоит на gen_hi;
//   MODE_SAW    (2) - пила: рост на gen_step за тик, после gen_hi (с учётом шага)
//                     сброс в gen_lo;
//   MODE_NOISE  (3) - шум: gen_lo + (случайное число & gen_noise_mask), но не больше gen_hi;
//                     случайные числа - 16-битный регистр LFSR (x^16+x^15+x^13+x^4+1).
// При смене режима, по импульсу restart и после сброса последовательность
// начинается заново с gen_lo (режим шума - с начального состояния LFSR).
//
// Выход temp_val - текущее значение, temp_word - то же в виде регистра температуры
// LM75A (11 бит в D15..D5, младшие 5 бит 0), он же Q8.8 из PROTOCOL.md.
// =============================================================================
module temp_gen #(
    parameter logic [15:0] LFSR_SEED = 16'hACE1
) (
    input  logic                clk,
    input  logic                rst_n,          // асинхронный сброс, активный 0

    input  logic                tick,           // импульс: следующее значение
    input  logic                restart,        // импульс: начать заново
    input  logic [1:0]          gen_mode,
    input  logic signed [10:0]  gen_lo,
    input  logic signed [10:0]  gen_hi,
    input  logic        [10:0]  gen_step,       // шаг роста, больше 0
    input  logic        [7:0]   gen_noise_mask, // амплитуда шума: маска случайного числа

    output logic signed [10:0]  temp_val,
    output logic        [15:0]  temp_word
);

    localparam logic [1:0] MODE_CONST  = 2'd0,
                           MODE_LINEAR = 2'd1,
                           MODE_SAW    = 2'd2,
                           MODE_NOISE  = 2'd3;

    logic [1:0]  mode_q;
    logic [15:0] lfsr;

    // вычисления в 13 битах со знаком: gen_step до 2047 не вызывает переполнения
    wire signed [12:0] lo_x    = 13'(gen_lo);
    wire signed [12:0] hi_x    = 13'(gen_hi);
    wire signed [12:0] next_x  = 13'(temp_val) + 13'(signed'({2'b0, gen_step}));
    wire        [7:0]  rnd_m   = lfsr[7:0] & gen_noise_mask;
    wire signed [12:0] noise_x = lo_x + 13'(signed'({5'b0, rnd_m}));

    wire fb        = lfsr[15] ^ lfsr[14] ^ lfsr[12] ^ lfsr[3];
    wire mode_chg  = (gen_mode != mode_q);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_q   <= MODE_CONST;
            temp_val <= '0;
            lfsr     <= LFSR_SEED;
        end else begin
            mode_q <= gen_mode;

            if (restart || mode_chg) begin
                temp_val <= gen_lo;
                lfsr     <= LFSR_SEED;
            end else if (tick) begin
                lfsr <= {lfsr[14:0], fb};
                case (gen_mode)
                    MODE_CONST:  temp_val <= gen_lo;
                    MODE_LINEAR: temp_val <= (next_x > hi_x) ? gen_hi : 11'(next_x);
                    MODE_SAW:    temp_val <= (next_x > hi_x) ? gen_lo : 11'(next_x);
                    default:     temp_val <= (noise_x > hi_x) ? gen_hi : 11'(noise_x);
                endcase
            end
        end
    end

    assign temp_word = {temp_val, 5'b0};

endmodule
