BAUD =      (1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800)
BIT_TICKS = (250,   125,   62,   52,    52,    52,    23,     23,     19,     21)
XTAL = 40000000

for baud, bit_ticks in zip(BAUD, BIT_TICKS):
    RMT_RES_HZ = baud * bit_ticks
    RMT_NS_PER_TICK = 1000000000 // RMT_RES_HZ
    CLK_DIV = XTAL // RMT_RES_HZ
    REAL_CLK = XTAL // CLK_DIV
    signal_range_min_ns = RMT_NS_PER_TICK // 20
    signal_range_max_ns = 10 * RMT_NS_PER_TICK * bit_ticks
    print(f"BAUD = {baud}")
    print(f"BIT_TICKS = {bit_ticks}")
    print(f"CLK_DIV = {CLK_DIV}  ({XTAL / RMT_RES_HZ})")
    print(f"REAL_CLK = {REAL_CLK} (DEV={100 * (REAL_CLK - RMT_RES_HZ) / RMT_RES_HZ:.2f}%)")
    print(f"RMT_RES_HZ = {RMT_RES_HZ}")
    print(f"RMT_NS_PER_TICK = {RMT_NS_PER_TICK}")
    print(f"signal_range_min_ns = {signal_range_min_ns}")
    print(f"signal_range_max_ns = {signal_range_max_ns}\n")
