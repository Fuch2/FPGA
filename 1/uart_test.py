"""
Проверка UART на плате.

Прошивка UART_top (эхо): программа отправляет байты в COM-порт адаптера
USB-TTL и сравнивает то, что вернула ПЛИС, с отправленным.
Прошивка UART_top2 (команды МУ и пакеты МО -> МУ, PROTOCOL.md раздел 5):
режим -p проверяет парсеры команд и пакетов и конвейер передатчика,
режим -m показывает всё, что передаёт ПЛИС (пакеты и текст).

Установка:   pip install pyserial
Запуск:      python uart_test.py            - список портов
             python uart_test.py COM7       - проверка эха (UART_top)
             python uart_test.py COM7 -i    - ручной режим: ввод строк и эхо
             python uart_test.py COM7 -w    - наблюдение: есть ли эхо (проверка RESET)
             python uart_test.py COM7 -p    - проверка UART_top2 (перед запуском нажмите KEY3)
             python uart_test.py COM7 -m    - наблюдение за UART_top2 (KEY1, KEY2, команды)
"""
import argparse
import os
import sys
import time

import serial
import serial.tools.list_ports


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("COM-порты не найдены. Подключите адаптер USB-TTL и установите драйвер.")
    else:
        print("Доступные порты:")
        for p in ports:
            print(f"  {p.device:8} {p.description}")
    print("\nЗапуск: python uart_test.py COMx")


def exchange(ser, data):
    """Отправить данные и прочитать столько же байт в ответ."""
    ser.reset_input_buffer()
    ser.write(data)
    return ser.read(len(data))


def check(name, sent, got):
    if got == sent:
        print(f"  [OK]   {name}: {len(sent)} байт")
        return True
    print(f"  [FAIL] {name}: отправлено {len(sent)}, принято {len(got)}")
    for i, (s, g) in enumerate(zip(sent, got)):
        if s != g:
            print(f"         первое расхождение: байт {i}, отправлен 0x{s:02X}, принят 0x{g:02X}")
            break
    if len(got) == 0:
        print("         ответа нет: проверьте подключение TX/RX, GND, прошивку и скорость")
    return False


def auto_test(ser):
    ok = True
    print("Проверка эха:")

    ok &= check("один байт 0x55", b"\x55", exchange(ser, b"\x55"))
    ok &= check("строка", b"Hello, FPGA!", exchange(ser, b"Hello, FPGA!"))
    ok &= check("все значения 0x00..0xFF", bytes(range(256)), exchange(ser, bytes(range(256))))

    # поток случайных данных блоками по 256 байт
    total, t0, stream_ok = 0, time.perf_counter(), True
    for _ in range(40):
        block = os.urandom(256)
        got = exchange(ser, block)
        total += len(got)
        if got != block:
            stream_ok = check("поток случайных байт", block, got)
            break
    dt = time.perf_counter() - t0
    if stream_ok:
        print(f"  [OK]   поток случайных байт: {total} байт, {total / dt:.0f} байт/с")
    ok &= stream_ok

    print()
    print("UART РАБОТАЕТ" if ok else "ЕСТЬ ОШИБКИ")
    return ok


def interactive(ser):
    print("Ручной режим: введите строку и нажмите Enter, пустая строка - выход.")
    while True:
        line = input("> ")
        if not line:
            break
        data = line.encode("utf-8")
        try:
            got = exchange(ser, data)
        except serial.SerialException as e:
            print(f"  адаптер не отвечает ({e}), переоткрываю порт...")
            if not reopen(ser):
                return
            continue
        if not got:
            print("< (нет ответа)")
        else:
            print("< " + got.decode("utf-8", errors="replace") + ("" if got == data else "   (не совпадает!)"))


def reopen(ser):
    """Закрыть и снова открыть порт после сбоя адаптера."""
    ser.close()
    time.sleep(0.5)
    try:
        ser.open()
        print("  порт снова открыт, повторите ввод")
        return True
    except serial.SerialException as e:
        print(f"  не удалось открыть порт ({e}). Переподключите адаптер USB-TTL и запустите программу снова.")
        return False


def watch(ser):
    """Отправлять байт 5 раз в секунду и сообщать, когда эхо пропадает и появляется."""
    print("Наблюдение: байт отправляется 5 раз в секунду.")
    print("Нажмите RESET, подержите пару секунд и отпустите. Выход - Ctrl+C.\n")
    ser.timeout = 0.1
    t0, n, state = time.perf_counter(), 0, None
    while True:
        b = bytes([n & 0xFF])
        n += 1
        ok = exchange(ser, b) == b
        if ok != state:
            print(f"  {time.perf_counter() - t0:6.1f} с   " + ("эхо есть" if ok else "нет ответа"))
            state = ok
        time.sleep(0.2)


# ---------------------------------------------------------------------------
# UART_top2: команды МУ и пакеты МО -> МУ (PROTOCOL.md, раздел 5)
# ---------------------------------------------------------------------------
SYNC = 0xAA
CMD_START = b"\x01"
CMD_STOP = b"\x02"
GEN_PERIOD = 0.1                # период пакетов генератора в UART_top2, с
MSG = b"UART_top2\r\n"          # строка по KEY2


def s16(w):
    """16-битный код -> число со знаком."""
    return w - 0x10000 if w & 0x8000 else w


def sat16(v):
    """Число со знаком -> 16-битный код с насыщением до -32768..32767."""
    return max(-32768, min(32767, v)) & 0xFFFF


def make_packet(meas, filt, err, chk_flip=0):
    """Пакет 0xAA + MEAS + FILT + ERR (старшим байтом вперёд) + XOR; chk_flip портит CHK."""
    data = bytes([meas >> 8, meas & 0xFF, filt >> 8, filt & 0xFF, err >> 8, err & 0xFF])
    chk = 0
    for b in data:
        chk ^= b
    return bytes([SYNC]) + data + bytes([chk ^ chk_flip])


def gen_packet(n):
    """Пакет номер n генератора test_pkt_gen: 25.0 °C + n * 0.125 °C, фильтр вдвое медленнее."""
    meas = (0x1900 + n * 0x20) & 0xFFFF
    filt = (0x1900 + n * 0x10) & 0xFFFF
    return meas, filt, sat16(s16(meas) - s16(filt))


def q88(w):
    """Q8.8 -> градусы."""
    return s16(w) / 256


class PacketStream:
    """Разбор потока от МО так же, как в МУ: поиск 0xAA, проверка CHK,
    при неверной CHK сдвиг на 1 байт. Байты вне пакетов копятся в text."""

    def __init__(self):
        self.buf = bytearray()
        self.packets = []           # (meas, filt, err)
        self.text = bytearray()
        self.bad = 0                # сдвигов из-за неверной CHK

    def feed(self, data):
        self.buf += data
        while self.buf:
            if self.buf[0] != SYNC:
                self.text.append(self.buf.pop(0))
            elif len(self.buf) < 8:
                break
            else:
                p = self.buf[:8]
                chk = 0
                for b in p[1:7]:
                    chk ^= b
                if chk == p[7]:
                    self.packets.append(((p[1] << 8) | p[2], (p[3] << 8) | p[4], (p[5] << 8) | p[6]))
                    del self.buf[:8]
                else:
                    self.bad += 1
                    self.text.append(self.buf.pop(0))


def read_for(ser, seconds, stream=None):
    """Читать всё, что приходит, в течение заданного времени."""
    data = bytearray()
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        data += ser.read(max(1, ser.in_waiting))
    if stream is not None:
        stream.feed(data)
    return bytes(data)


def read_n(ser, n, timeout):
    """Прочитать n байт, но не дольше timeout секунд."""
    data = bytearray()
    end = time.perf_counter() + timeout
    while len(data) < n and time.perf_counter() < end:
        data += ser.read(n - len(data))
    return bytes(data)


def report(name, ok, why=""):
    print(f"  [{'OK' if ok else 'FAIL'}]{' ' if ok else ''} {name}" + ("" if ok or not why else f": {why}"))
    return ok


def proto_test(ser):
    ok = True
    expect = {"r": 0, "t": 0, "E": 0, "u": 0}    # ожидаемые счётчики индикатора
    ser.timeout = 0.05
    print("Проверка UART_top2: парсер команд, парсер пакетов, конвейер передатчика")

    # исходное состояние: генератор остановлен, входной буфер пуст
    ser.write(CMD_STOP)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # 1. мусор -------------------------------------------------------------
    junk = bytes([0x00, 0x03, 0x55, 0xFF, 0x10, 0x7F, 0x80, 0xFE])
    ser.write(junk)
    expect["u"] += len(junk)
    got = read_for(ser, 0.5)
    ok &= report(f"мусор ({len(junk)} байт) игнорируется", not got, f"ПЛИС передала {len(got)} байт")

    # 2. Старт / Стоп -------------------------------------------------------
    stream = PacketStream()
    ser.write(CMD_START)
    read_for(ser, 1.05, stream)
    n_run = len(stream.packets)
    ser.write(CMD_STOP)
    read_for(ser, 0.3, stream)
    n_total = len(stream.packets)
    after_stop = read_for(ser, 0.5)
    expect["t"] += n_total
    values_ok = all(p == gen_packet(i) for i, p in enumerate(stream.packets))
    ok &= report(f"Старт: {n_run} пакетов за 1.05 с (ожидается 10..12), значения по порядку с n = 0",
                 9 <= n_run <= 13 and values_ok and stream.bad == 0 and not stream.text,
                 f"плохих CHK {stream.bad}, лишних байт {len(stream.text)}, значения {'OK' if values_ok else 'не совпадают'}")
    ok &= report(f"Стоп: после команды не больше 1 пакета ({n_total - n_run}), затем тишина",
                 n_total - n_run <= 1 and not after_stop, f"после паузы ещё {len(after_stop)} байт")
    if stream.packets:
        m, f, e = stream.packets[-1]
        print(f"         последний пакет: MEAS {q88(m):.3f} °C, FILT {q88(f):.3f} °C, ERR {q88(e):.3f} °C")

    # 3. эхо верных пакетов -------------------------------------------------
    special = [(0x0102, 0x0201, 0x0102),        # команды Старт/Стоп внутри пакета
               (0xAAAA, 0xAAAA, 0xAAAA),        # синхробайт внутри пакета
               (0x1960, 0x1900, 0x0060),        # пример из PROTOCOL.md
               (0x0000, 0x0000, 0x0000),
               (0xFFFF, 0xFFFF, 0xFFFF)]
    pkts = special + [tuple(int.from_bytes(os.urandom(2), "big") for _ in range(3)) for _ in range(195)]
    echo_ok, n_sent, n_echo = True, 0, 0
    for i in range(0, len(pkts), 25):
        data = b"".join(make_packet(*p) for p in pkts[i:i + 25])
        ser.write(data)
        n_sent += len(data) // 8
        got = read_n(ser, len(data), 2.0)
        n_echo += len(got) // 8
        if got != data:
            echo_ok = check(f"эхо пакетов {i}..{i + 24}", data, got)
            break
    expect["r"] += n_sent               # ПЛИС приняла все отправленные пакеты
    expect["t"] += n_echo
    expect["E"] += n_sent - n_echo      # потерянное эхо = переполнение очереди
    ok &= report(f"эхо {len(pkts)} верных пакетов (в том числе с 0x01, 0x02, 0xAA внутри)", echo_ok)
    got = read_for(ser, 0.3)
    ok &= report("0x01 внутри пакета не запускает генератор", not got, f"ПЛИС передала ещё {len(got)} байт")

    # 4. неверная CHK -------------------------------------------------------
    bad = b"".join(make_packet(*gen_packet(100 + i), chk_flip=1 << (i % 8)) for i in range(20))
    ser.write(bad)
    expect["E"] += 20
    got = read_for(ser, 0.3)
    ok &= report("20 пакетов с неверной CHK: эха нет", not got, f"ПЛИС передала {len(got)} байт")

    # 5. оборванный пакет ---------------------------------------------------
    good = make_packet(0x1234, 0x5678, 0x9ABC)
    ser.write(make_packet(0x1111, 0x2222, 0x3333)[:4])
    time.sleep(0.05)                                    # больше тайм-аута 1 мс
    ser.write(good)
    got = read_n(ser, 8, 1.0) + read_for(ser, 0.2)
    expect["E"] += 1
    expect["r"] += 1
    expect["t"] += len(got) // 8
    ok &= report("оборванный пакет отброшен, следующий принят", got == good,
                 f"принято {got.hex(' ')}")

    # 6. замер: длинные пачки вплотную ------------------------------------------
    print("  [INFO] длинные пачки вплотную (скорость адаптера против скорости ПЛИС):")
    for n in (50, 100, 200):            # сумма r и t остаётся меньше 1000
        sent = [tuple(int.from_bytes(os.urandom(2), "big") for _ in range(3)) for _ in range(n)]
        ser.write(b"".join(make_packet(*p) for p in sent))
        stream = PacketStream()
        read_for(ser, n * 8 * 10 / 115200 + 0.3, stream)
        lost, k = [], 0                 # номера потерянных пакетов
        for i, p in enumerate(sent):
            if k < len(stream.packets) and stream.packets[k] == p:
                k += 1
            else:
                lost.append(i)
        expect["r"] += n
        expect["t"] += len(stream.packets)
        expect["E"] += len(lost)
        where = f", первый потерян №{lost[0]}" if lost else ""
        print(f"         пачка {n:3d} пакетов: эхо {len(stream.packets)}, потеряно {len(lost)}{where}")
    print("         потери в длинных пачках - не ошибка: адаптер передаёт чуть быстрее ПЛИС,")
    print("         очередь эха лишь отодвигает момент переполнения (чем позже - тем ближе скорости)")

    # 7. Старт + эхо одновременно -------------------------------------------
    stream = PacketStream()
    sent = [tuple(int.from_bytes(os.urandom(2), "big") for _ in range(3)) for _ in range(30)]
    ser.write(CMD_START)
    for p in sent:
        ser.write(make_packet(*p))
        read_for(ser, 0.02, stream)
    read_for(ser, 0.3, stream)
    ser.write(CMD_STOP)
    read_for(ser, 0.3, stream)
    n_echo = n_gen = n_unknown = 0
    for p in stream.packets:
        if n_echo < len(sent) and p == sent[n_echo]:
            n_echo += 1
        elif p == gen_packet(n_gen):
            n_gen += 1
        else:
            n_unknown += 1
    mixed_ok = stream.bad == 0 and not stream.text and n_unknown == 0
    expect["r"] += len(sent)
    expect["t"] += len(stream.packets)
    ok &= report(f"Старт + эхо: эхо {n_echo} из {len(sent)}, пакетов генератора {n_gen}, не перемешаны",
                 mixed_ok and n_echo == len(sent) and n_gen >= 3,
                 f"плохих CHK {stream.bad}, лишних байт {len(stream.text)}, неизвестных пакетов {n_unknown}")

    print()
    print("UART_top2 РАБОТАЕТ" if ok else "ЕСТЬ ОШИБКИ")
    print()
    print("Если перед проверкой нажимали KEY3, на индикаторе (переключение - KEY4) должно быть:")
    print(f"  r{expect['r'] % 1000:3d}   t{expect['t'] % 1000:3d}   E{expect['E'] % 1000:3d}   "
          f"u{expect['u'] % 1000:3d}   (счётчики идут по кругу 0..999)")
    print("  в режиме «b XX» - последний принятый байт 02 (команда Стоп);")
    print("  led[3] горит (были ошибки), led[2] погашен (Стоп).")
    return ok


def monitor(ser):
    """Показывать пакеты и текст от ПЛИС; клавиши s/x - Старт/Стоп, q - выход."""
    try:
        import msvcrt               # Windows: чтение клавиш без Enter
    except ImportError:
        msvcrt = None
    print("Наблюдение за UART_top2. KEY1 на плате - пакет, KEY2 - строка.")
    if msvcrt:
        print("Клавиши: s - Старт (0x01), x - Стоп (0x02), q - выход.")
    else:
        print("Выход - Ctrl+C.")
    ser.timeout = 0.1
    stream = PacketStream()
    shown = 0
    while True:
        stream.feed(ser.read(max(1, ser.in_waiting)))
        while shown < len(stream.packets):
            m, f, e = stream.packets[shown]
            shown += 1
            print(f"  пакет {shown:4d}: MEAS {m:04X} ({q88(m):8.3f} °C)  FILT {f:04X} ({q88(f):8.3f} °C)"
                  f"  ERR {e:04X} ({q88(e):8.3f} °C)")
        if stream.text:
            print(f"  текст: {bytes(stream.text)!r}")
            stream.text.clear()
        if stream.bad:
            print(f"  неверная CHK: {stream.bad}")
            stream.bad = 0
        if msvcrt and msvcrt.kbhit():
            k = msvcrt.getwch().lower()
            if k == "s":
                ser.write(CMD_START)
                print("  -> Старт")
            elif k == "x":
                ser.write(CMD_STOP)
                print("  -> Стоп")
            elif k == "q":
                return


def main():
    ap = argparse.ArgumentParser(description="Проверка UART на плате эхом")
    ap.add_argument("port", nargs="?", help="COM-порт адаптера, например COM7")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="скорость (по умолчанию 115200)")
    ap.add_argument("-i", "--interactive", action="store_true", help="ручной режим")
    ap.add_argument("-w", "--watch", action="store_true", help="наблюдение за эхом (проверка RESET)")
    ap.add_argument("-p", "--proto", action="store_true", help="проверка UART_top2: команды, пакеты, конвейер")
    ap.add_argument("-m", "--monitor", action="store_true", help="наблюдение за UART_top2: пакеты и текст")
    args = ap.parse_args()

    if not args.port:
        list_ports()
        return 0

    try:
        ser = serial.serial_for_url(args.port, baudrate=args.baud, bytesize=8,
                                    parity="N", stopbits=1, timeout=1)
    except serial.SerialException as e:
        print(f"Не удалось открыть {args.port}: {e}")
        return 1

    with ser:
        print(f"Порт {args.port}, {args.baud} бод, 8N1\n")
        try:
            if args.proto:
                return 0 if proto_test(ser) else 1
            if args.monitor:
                monitor(ser)
                return 0
            if args.watch:
                watch(ser)
            if args.interactive:
                interactive(ser)
                return 0
            return 0 if auto_test(ser) else 1
        except (KeyboardInterrupt, EOFError):
            print("\nВыход.")
            return 0
        except serial.SerialException as e:
            print(f"\nАдаптер перестал отвечать: {e}")
            print("Переподключите адаптер USB-TTL и запустите программу снова.")
            return 1


if __name__ == "__main__":
    sys.exit(main())
