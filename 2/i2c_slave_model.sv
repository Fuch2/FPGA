// =============================================================================
// Поведенческая модель I2C-ведомого с регистрами (только для симуляции)
//
// Память mem[0 .. 2**MEM_BITS-1] и указатель ptr. Ведомый отвечает ACK на адрес,
// совпавший с ADDR по битам, где ADDR_MASK = 1 (AT24C08 занимает 4 соседних адреса:
// ADDR = 0x50, ADDR_MASK = 0x7C, BLOCK_BITS = 2 - младшие 2 бита адреса идут в
// старшие биты адреса ячейки).
//   запись:  первый байт после адреса - указатель (младшие 8 бит ptr), остальные
//            пишутся в mem[ptr] с автоинкрементом; при PAGE_SIZE > 0 адрес
//            переворачивается внутри страницы записи;
//   чтение:  с текущего указателя, автоинкремент по кругу по всей памяти;
//   указатель сохраняется между транзакциями (как «текущий адрес» EEPROM и
//   указатель LM75A).
// После STOP, закончившего запись, ведомый не отвечает WRITE_CYCLE_NS (внутренний
// цикл записи EEPROM: NACK на адрес, опрос ACK).
//
// Управление из тестбенча (обращением по иерархии):
//   stretch_ns  - после каждого спада SCL ведомый удерживает SCL в 0 столько нс;
//   wr_limit    - сколько байт после адреса принять за транзакцию (дальше NACK);
//   hold_sda    - SDA залипла в 0;
//   absent      - ведомый «не припаян»: на адрес не отвечает.
// Журналы: n_start, n_stop, n_addr_ack, n_addr_nack, n_master_nack,
//   rx_log (байты, принятые после адреса, включая указатель), tx_log (отданные).
// =============================================================================
`timescale 1ns/1ps

module i2c_slave_model #(
    parameter logic [6:0] ADDR            = 7'h48,
    parameter logic [6:0] ADDR_MASK       = 7'h7F,
    parameter int         MEM_BITS        = 8,
    parameter int         BLOCK_BITS      = 0,
    parameter int         PAGE_SIZE       = 0,
    parameter real        WRITE_CYCLE_NS  = 0.0
) (
    inout tri1 scl,
    inout tri1 sda
);

    localparam int MEM_SIZE = 1 << MEM_BITS;

    logic [7:0] mem [MEM_SIZE];
    int         ptr = 0;

    // управление
    int  stretch_ns = 0;
    int  wr_limit   = 1_000_000;
    bit  hold_sda   = 1'b0;
    bit  absent     = 1'b0;

    // журналы
    int n_start = 0, n_stop = 0, n_addr_ack = 0, n_addr_nack = 0, n_master_nack = 0;
    logic [7:0] rx_log [$];
    logic [7:0] tx_log [$];

    // линии: открытый сток
    logic sda_o_low = 1'b0, scl_o_low = 1'b0;

    assign sda = (sda_o_low || hold_sda) ? 1'b0 : 1'bz;
    assign scl = scl_o_low ? 1'b0 : 1'bz;

    typedef enum {S_IDLE, S_ADDR, S_ACK_DRV, S_ACK_REL, S_WR, S_RD, S_RD_ACK, S_IGNORE} state_t;

    state_t     state = S_IDLE, after_ack = S_IGNORE;
    logic       scl_q = 1'b1, sda_q = 1'b1;
    int         bit_cnt = 0;
    logic [7:0] sh = '0;
    logic [7:0] tx_byte = '0;
    bit         is_read = 0, ack_out = 0, first_byte = 0, master_nack = 0, wrote = 0;
    int         wr_count = 0, blk = 0;
    realtime    busy_until = 0;

    initial for (int i = 0; i < MEM_SIZE; i++) mem[i] = '0;

    function automatic void advance_write();
        if (PAGE_SIZE > 0) ptr = (ptr / PAGE_SIZE) * PAGE_SIZE + ((ptr + 1) % PAGE_SIZE);
        else               ptr = (ptr + 1) % MEM_SIZE;
    endfunction

    function automatic void load_tx();
        tx_byte   = mem[ptr];
        tx_log.push_back(tx_byte);
        ptr       = (ptr + 1) % MEM_SIZE;
        state     = S_RD;
        bit_cnt   = 1;
        sda_o_low = !tx_byte[7];
    endfunction

    function automatic void do_start();
        n_start++;
        state     = S_ADDR;
        bit_cnt   = 0;
        sh        = '0;
        wr_count  = 0;
        sda_o_low = 1'b0;
    endfunction

    function automatic void do_stop();
        n_stop++;
        if (wrote) begin
            busy_until = $realtime + WRITE_CYCLE_NS;
            wrote      = 1'b0;
        end
        state     = S_IDLE;
        sda_o_low = 1'b0;
    endfunction

    // SCL вырос: выборка бита мастера
    function automatic void do_rise();
        case (state)
            S_ADDR: begin
                sh = {sh[6:0], sda};
                bit_cnt++;
                if (bit_cnt == 8) begin
                    is_read = sh[0];
                    if (((sh[7:1] ^ ADDR) & ADDR_MASK) == 0 && !absent && $realtime >= busy_until) begin
                        ack_out = 1'b1;
                        n_addr_ack++;
                        blk = (BLOCK_BITS > 0) ? ((sh >> 1) & ((1 << BLOCK_BITS) - 1)) : 0;
                        ptr = (blk * 256 + (ptr % 256)) % MEM_SIZE;
                        after_ack  = is_read ? S_RD : S_WR;
                        first_byte = !is_read;
                    end else begin
                        ack_out = 1'b0;
                        n_addr_nack++;
                        after_ack = S_IGNORE;
                    end
                    state = S_ACK_DRV;
                end
            end
            S_WR: begin
                sh = {sh[6:0], sda};
                bit_cnt++;
                if (bit_cnt == 8) begin
                    rx_log.push_back(sh);
                    if (wr_count >= wr_limit) begin
                        ack_out   = 1'b0;
                        after_ack = S_IGNORE;
                    end else begin
                        ack_out   = 1'b1;
                        after_ack = S_WR;
                        wr_count++;
                        if (first_byte) begin
                            ptr        = (blk * 256 + sh) % MEM_SIZE;
                            first_byte = 1'b0;
                        end else begin
                            mem[ptr] = sh;
                            wrote    = 1'b1;
                            advance_write();
                        end
                    end
                    state = S_ACK_DRV;
                end
            end
            S_RD_ACK: begin
                master_nack = sda;
                if (sda) n_master_nack++;
            end
            default: ;
        endcase
    endfunction

    // SCL упал: можно менять SDA
    function automatic void do_fall();
        case (state)
            S_ACK_DRV: begin
                bit_cnt = 0;
                if (ack_out) begin
                    sda_o_low = 1'b1;
                    state     = S_ACK_REL;
                end else begin
                    state = S_IGNORE;
                end
            end
            S_ACK_REL: begin
                sda_o_low = 1'b0;
                bit_cnt   = 0;
                sh        = '0;
                state     = after_ack;
                if (state == S_RD) load_tx();
            end
            S_RD: begin
                if (bit_cnt == 8) begin
                    sda_o_low = 1'b0;
                    state     = S_RD_ACK;
                end else begin
                    sda_o_low = !tx_byte[7 - bit_cnt];
                    bit_cnt++;
                end
            end
            S_RD_ACK: begin
                if (!master_nack) load_tx();
                else              state = S_IGNORE;
            end
            default: ;
        endcase
    endfunction

    always @(scl or sda) begin
        if (!hold_sda) begin
            if      (scl === 1'b1 && sda_q === 1'b1 && sda === 1'b0) do_start();
            else if (scl === 1'b1 && sda_q === 1'b0 && sda === 1'b1) do_stop();
            else if (scl === 1'b1 && scl_q === 1'b0)                 do_rise();
            else if (scl === 1'b0 && scl_q === 1'b1)                 do_fall();
        end
        scl_q = scl;
        sda_q = sda;
    end

    // растяжка тактов: ведомый не отпускает SCL после спада
    always @(negedge scl) begin
        if (stretch_ns > 0 && state != S_IDLE) begin
            scl_o_low = 1'b1;
            #(stretch_ns);
            scl_o_low = 1'b0;
        end
    end

endmodule
