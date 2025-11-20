from machine import Pin, SoftI2C
from libs.tcs3472_micropython.tcs3472 import tcs3472
from utime import sleep

gp21 = Pin(21, Pin.OUT)
gp21.value(1)

def fast_read_raw(i2c, addr=0x29):
    data = i2c.readfrom_mem(addr, 0x14 | 0x80, 8)  # COMMAND bit required!

    c = data[0] | (data[1] << 8)
    r = data[2] | (data[3] << 8)
    g = data[4] | (data[5] << 8)
    b = data[6] | (data[7] << 8)

    return c, r, g, b

def test_tcs3472():
    i2c_bus = SoftI2C(sda=Pin(8), scl=Pin(9))

    # MUST initialize the sensor
    tcs = tcs3472(i2c_bus)

    r_sum = g_sum = b_sum = 0
    total = 10000
    while True:
        for _ in range(total):
            c, r, g, b = fast_read_raw(i2c_bus)
            r_sum += r
            g_sum += g
            b_sum += b

        total_rgb = r_sum + g_sum + b_sum

        if total_rgb == 0:
            return (0, 0, 0)

        return (
            r_sum / total_rgb,
            g_sum / total_rgb,
            b_sum / total_rgb
        )

if __name__ == "__main__":
    avg_r, avg_g, avg_b = test_tcs3472()
    print("Ratio RGB:", avg_r, avg_g, avg_b)

