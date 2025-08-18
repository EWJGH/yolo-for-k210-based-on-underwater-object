import os
from board import board_info
from fpioa_manager import fm
from maix import GPIO
import time
import lcd
import image
import machine

fm.register(board_info.KEY0, fm.fpioa.GPIOHS0)
fm.register(board_info.KEY1, fm.fpioa.GPIOHS1)
fm.register(board_info.KEY2, fm.fpioa.GPIOHS2)
fm.register(board_info.BEEP, fm.fpioa.GPIO2)
key0 = GPIO(GPIO.GPIOHS0, GPIO.IN, GPIO.PULL_UP)
key1 = GPIO(GPIO.GPIOHS1, GPIO.IN, GPIO.PULL_UP)
key2 = GPIO(GPIO.GPIOHS2, GPIO.IN, GPIO.PULL_UP)
beep = GPIO(GPIO.GPIO2, GPIO.OUT, GPIO.PULL_NONE, value=0)

key0_en = False
key1_en = False
key2_en = False
def key_irq_handler(key):
    global key0
    global key1
    global key2
    global key0_en
    global key1_en
    global key2_en
    time.sleep_ms(20)
    if key is key0 and key.value() == 0:
        key0_en = True
    if key is key1 and key.value() == 0:
        key1_en = True
    if key is key2 and key.value() == 0:
        key2_en = True

key0.irq(key_irq_handler, GPIO.IRQ_FALLING, GPIO.WAKEUP_NOT_SUPPORT, 7)
key1.irq(key_irq_handler, GPIO.IRQ_FALLING, GPIO.WAKEUP_NOT_SUPPORT, 7)
key2.irq(key_irq_handler, GPIO.IRQ_FALLING, GPIO.WAKEUP_NOT_SUPPORT, 7)

lcd.init()
WIDTH = lcd.width()
HEIGHT = lcd.height()

fb = image.Image(size=(WIDTH, HEIGHT))
fb.draw_rectangle(0, 0, WIDTH, HEIGHT, (162, 202, 216), 0, True)

pic_beep_on = image.Image("/sd/sample/resource/picture/beep/beep_on.bmp")

pic_beep_off = image.Image("/sd/sample/resource/picture/beep/beep_off.bmp")
fb.draw_image(pic_beep_off, (WIDTH - pic_beep_off.width()) // 2, (HEIGHT - pic_beep_off.height()) // 2)

lcd.display(fb)

while True:
    if key0_en is True or key1_en is True or key2_en is True:
        if key0_en is True:
            key0_en = False
            beep.value(0)
        if key1_en is True:
            key1_en = False
            beep.value(0)
            try:
                with open("/sd/main.py", "rb") as f:
                    os.remove("/sd/main.py")
            except Exception as e:
                pass
            with open("/sd/sample/app/launcher.py", "rb") as f:
                code_src = f.read()
            with open("/sd/main.py", "wb") as f:
                f.write(code_src)
            machine.reset()
        if key2_en is True:
            key2_en = False
            beep.value(1)

        fb.draw_rectangle(0, 0, WIDTH, HEIGHT, (162, 202, 216), 0, True)

        if beep.value() == 0:
            fb.draw_image(pic_beep_off, (WIDTH - pic_beep_off.width()) // 2, (HEIGHT - pic_beep_off.height()) // 2)
        else:
            fb.draw_image(pic_beep_on, (WIDTH - pic_beep_on.width()) // 2, (HEIGHT - pic_beep_on.height()) // 2)

        lcd.display(fb)
