import neopixel
from machine import Pin
from time import ticks_ms, ticks_diff

# NeoPixel setup
lights = neopixel.NeoPixel(Pin(15), 2)

# button setup
btn = Pin(34, Pin.IN, Pin.PULL_UP) 
DEBOUNCE_MS = 200
last_press = 0

state = 0  # 0 = blinking plane lights, 1 = hue cycle

def button_handler(pin):
    global last_press, state
    curr_time = ticks_ms()
    if ticks_diff(curr_time, last_press) > DEBOUNCE_MS:
        if pin.value() == 0:
            last_press = curr_time
            state = 1 - state  # toggle state
            print("Button pressed, State:", state)

btn.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)

# Color wheel helper function was pulled from the
# internet because I didn't know how to hue shifting
def color_wheel(pos):
    pos = pos % 256
    if pos < 85:
        return (pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return (0, pos * 3, 255 - pos * 3)

# Main loop
last_update_time = 0
blink_interval = 1000  # ms, for state 0
hue_interval = 30      # ms, for state 1

blink_on = True
hue = 0

while True:
    curr_time = ticks_ms()

    if state == 0:
        if ticks_diff(curr_time, last_update_time) >= blink_interval:
            last_update_time = curr_time
            blink_on = not blink_on
            if blink_on:
                lights[0] = (0, 20, 2)   # green
                lights[1] = (0, 0, 0)    # off
            else:
                lights[0] = (0, 0, 0)    # off
                lights[1] = (20, 0, 0)   # red
            lights.write()

    else:
        if ticks_diff(curr_time, last_update_time) >= hue_interval:
            last_update_time = curr_time
            color = color_wheel(hue)
            lights[0] = color * (20/256)
            lights[1] = color * (20/256)
            lights.write()
            hue = (hue + 1) % 256