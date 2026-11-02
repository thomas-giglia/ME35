# i2c_check.py
# Diagnoses why the color sensor isn't found. Run it in Thonny and
# send back everything it prints.

from machine import Pin, I2C, SoftI2C
import time

PIN_SETS = ((16, 17), (32, 33))   # Grove 1 and Grove 5

for a, b in PIN_SETS:
    print("=== Pins %d and %d ===" % (a, b))

    # 1) Idle line levels: both should read 1 if pull-ups are present
    pa = Pin(a, Pin.IN)
    pb = Pin(b, Pin.IN)
    time.sleep_ms(5)
    print("Idle levels without pull-ups: D%d=%d  D%d=%d" % (a, pa.value(), b, pb.value()))
    if pa.value() == 0 or pb.value() == 0:
        print("  -> A line is stuck low: no pull-up resistors, no power,")
        print("     or a damaged sensor/cable.")

    # 2) Scan with hardware I2C, both pin orders, at a slow speed
    for scl, sda in ((a, b), (b, a)):
        try:
            found = I2C(0, scl=Pin(scl), sda=Pin(sda), freq=100000).scan()
        except Exception as err:
            found = "error: %s" % err
        print("Hardware I2C  scl=D%d sda=D%d ->" % (scl, sda),
              [hex(x) for x in found] if isinstance(found, list) else found)

    # 3) Scan with software I2C, using the ESP32's internal pull-ups
    for scl, sda in ((a, b), (b, a)):
        Pin(scl, Pin.OPEN_DRAIN, Pin.PULL_UP)
        Pin(sda, Pin.OPEN_DRAIN, Pin.PULL_UP)
        try:
            found = SoftI2C(scl=Pin(scl, pull=Pin.PULL_UP),
                            sda=Pin(sda, pull=Pin.PULL_UP), freq=50000).scan()
        except Exception as err:
            found = "error: %s" % err
        print("Software I2C  scl=D%d sda=D%d ->" % (scl, sda),
              [hex(x) for x in found] if isinstance(found, list) else found)
    print()

print("What the results mean:")
print("  0x29 found         -> TCS34725, the chip the code expects")
print("  other address      -> a different sensor chip; tell Claude the address")
print("                        and the name printed on the sensor board")
print("  only found by the  -> missing pull-up resistors (the code can")
print("  software scan         be switched to use internal pull-ups)")
print("  nothing anywhere   -> sensor not powered or damaged")