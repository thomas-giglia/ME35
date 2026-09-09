import neopixel
from machine import Pin
lights = neopixel.NeoPixel(Pin(15),2)
lights[0] = (5,20,0)
lights[1] = (20,0,0)
lights.write()