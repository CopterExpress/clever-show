from __future__ import print_function
import threading
import time
from rpi_ws281x import *
from tasking_lib import wait as wait_until
import logging
logger = logging.getLogger(__name__)
# LED strip configuration:
LED_COUNT = 60 # Number of LED pixels.
LED_PIN = 21  # GPIO pin connected to the pixels (18 uses PWM!) (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10  # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False  # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0  # Set to '1' for GPIOs 13, 19, 41, 45 or 53

# define led strip
strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)

# variables
mode = ""
r = 0
g = 0
b = 0

r_prev = 0
g_prev = 0
b_prev = 0

direct = False
l = 0
wait_ms = 5.0

INTERRUPTER = threading.Event()
INTERRUPTER_UNSET = threading.Event()

def delay(delay_time, interrupter=INTERRUPTER, maxsleep=0.01):
    global mode
    wait_until(time.time()+delay_time, interrupter, maxsleep)
    if interrupter.is_set():
        mode = "off"


# functions
def math_wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return Color(pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return Color(255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return Color(0, pos * 3, 255 - pos * 3)


def rainbow(wait=10, direction=False, interrupter=INTERRUPTER):
    global wait_ms, direct, mode, INTERRUPTER
    wait_ms = wait
    direct = direction
    mode = "rainbow"
    INTERRUPTER=interrupter


def fill(red, green, blue, interrupter=INTERRUPTER):
    global r, g, b, mode, INTERRUPTER
    r = red
    g = green
    b = blue
    mode = "fill"
    INTERRUPTER=interrupter


def blink(red, green, blue, wait=250, interrupter=INTERRUPTER):
    global r, g, b, wait_ms, mode, INTERRUPTER
    r = red
    g = green
    b = blue
    wait_ms = wait
    mode = "blink"
    INTERRUPTER=interrupter


def chase(red, green, blue, wait=50, direction=False, interrupter=INTERRUPTER):
    global r, g, b, wait_ms, direct, mode, INTERRUPTER
    r = red
    g = green
    b = blue
    wait_ms = wait
    direct = direction
    mode = "chase"
    INTERRUPTER=interrupter


def wipe_to(red, green, blue, wait=50, direction=False, interrupter=INTERRUPTER):
    global r, g, b, wait_ms, direct, mode, INTERRUPTER
    r = red
    g = green
    b = blue
    wait_ms = wait
    direct = direction
    mode = "wipe_to"
    INTERRUPTER=interrupter


def fade_to(red, green, blue, wait=20, interrupter=INTERRUPTER):  # do not working with rainbow (solid colors only)
    global r, g, b, r_prev, g_prev, b_prev, wait_ms, mode, INTERRUPTER
    r_prev = r
    g_prev = g
    b_prev = b
    r = red
    g = green
    b = blue
    wait_ms = wait
    mode = "fade_to"
    INTERRUPTER=interrupter


def run(red, green, blue, length=strip.numPixels(), direction=False, wait=25, interrupter=INTERRUPTER):
    global r, g, b, l, wait_ms, direct, mode, INTERRUPTER
    r = red
    g = green
    b = blue
    l = length
    direct = direction
    wait_ms = wait
    mode = "run"
    INTERRUPTER=interrupter


def off():
    global mode
    mode = "off"


def strip_set(color):
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
    strip.show()


def strip_rainbow_frame(iteration, direction):
    for i in range(strip.numPixels()):
        n = ((strip.numPixels()-1)*direction) - i
        strip.setPixelColor(abs(n), math_wheel((int(i * 256 / strip.numPixels()) + iteration) & 255))
    strip.show()


def strip_chase_step(color, direction, interrupter=INTERRUPTER):
    for q in range(3):
        for i in range(0, strip.numPixels(), 3):
            n = ((strip.numPixels() - 1) * direction) - (i + q)
            strip.setPixelColor(abs(n), color)
        strip.show()
        delay(wait_ms / 1000.0, interrupter)
        for i in range(0, strip.numPixels(), 3):
            n = ((strip.numPixels() - 1) * direction) - (i + q)
            strip.setPixelColor(abs(n), 0)


def strip_wipe(color, direction, interrupter=INTERRUPTER):
    for i in range(strip.numPixels()):
        n = ((strip.numPixels() - 1) * direction) - i
        strip.setPixelColor(abs(n), color)
        delay(wait_ms / 1000.0, interrupter)
        if interrupter.is_set():
           return
        strip.show()


def strip_fade(r1, g1, b1, r2, g2, b2, frames=50, interrupter=INTERRUPTER):
    r_delta = (r2-r1)//frames
    g_delta = (g2-g1)//frames
    b_delta = (b2-b1)//frames
    for i in range(frames):
        red = r1 + (r_delta * i)
        green = g1 + (g_delta * i)
        blue = b1 + (b_delta * i)
        strip_set(Color(red, green, blue))
        delay(wait_ms / 1000.0, interrupter)
        if interrupter.is_set():
            return
    strip_set(Color(r2, g2, b2))


def strip_run_step(red, green, blue, length, direction, iteration):
    r_delta = red // length
    g_delta = green // length
    b_delta = blue // length
    direction = not direction
    for i in range(strip.numPixels()):
        n = ((strip.numPixels()-1)*direction)-((i+iteration) % strip.numPixels())
        r_fin = max(0, (red - (r_delta * i)))
        g_fin = max(0, (green - (g_delta * i)))
        b_fin = max(0, (blue - (b_delta * i)))
        strip.setPixelColor(abs(n), Color(r_fin, g_fin, b_fin))
    strip.show()


def strip_off():
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, Color(0, 0, 0))
    strip.show()


def led_thread():
    global mode
    logger.info("Starting LedLib thread")
    iteration = 0
    while True:
        if mode == "rainbow":
            if iteration >= 256:
                iteration = 0
            strip_rainbow_frame(iteration, direct)
            delay(wait_ms / 1000.0, INTERRUPTER)
            iteration += 1
        elif mode == "fill":
            strip_set(Color(r, g, b))
            mode = ""
        elif mode == "blink":
            strip_set(Color(r, g, b))
            delay(wait_ms / 1000.0, INTERRUPTER)
            strip_set(Color(0, 0, 0))
            delay(wait_ms / 1000.0, INTERRUPTER)
        elif mode == "chase":
            strip_chase_step(Color(r, g, b), direct)
        elif mode == "wipe_to":
            strip_wipe(Color(r, g, b,), direct, INTERRUPTER)
            mode = "fill"
        elif mode == "fade_to":
            strip_fade(r_prev, g_prev, b_prev, r, g, b, interrupter=INTERRUPTER)
            mode = ""
        elif mode == "run":
            strip_run_step(r, g, b, l, direct, iteration)
            delay(wait_ms / 1000.0, INTERRUPTER)
            iteration += 1
        elif mode == "off":
            strip_off()
            mode = ""
        else:
            delay(1 / 1000.0, interrupter=INTERRUPTER_UNSET)


# init
def init_led(led_pin = LED_PIN):
    global strip
    strip = Adafruit_NeoPixel(LED_COUNT, led_pin, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    strip.begin()
    t_l = threading.Thread(target=led_thread)
    t_l.daemon = True
    t_l.start()


if __name__ == '__main__':
    init_led()
    try:
        rainbow()
    except KeyboardInterrupt:
        off()
