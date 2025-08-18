import sys
sys.path.append("/sd/sample/resource/pymodule/font")
import os
from board import board_info
from fpioa_manager import fm
from maix import GPIO
import time
import lcd
import image
import machine
import sensor
import gc
import time
import font

fm.register(board_info.KEY1, fm.fpioa.GPIOHS1)
fm.register(board_info.KEY2, fm.fpioa.GPIOHS2)
key1 = GPIO(GPIO.GPIOHS1, GPIO.IN, GPIO.PULL_UP)
key2 = GPIO(GPIO.GPIOHS2, GPIO.IN, GPIO.PULL_UP)

key1_en = False
key2_en = False
def key_irq_handler(key):
    global key1
    global key2
    global key1_en
    global key2_en
    time.sleep_ms(20)
    if key is key1 and key.value() == 0:
        key1_en = True
    if key is key2 and key.value() == 0:
        key2_en = True

key1.irq(key_irq_handler, GPIO.IRQ_FALLING, GPIO.WAKEUP_NOT_SUPPORT, 7)
key2.irq(key_irq_handler, GPIO.IRQ_FALLING, GPIO.WAKEUP_NOT_SUPPORT, 7)

sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
sensor.set_vflip(True)

try:
    os.mkdir("/sd/sample/media/photo")
except Exception:
    pass

lcd.init()
WIDTH = lcd.width()
HEIGHT = lcd.height()

while True:
    img = sensor.snapshot()

    if key1_en is True or key2_en is True:
        if key1_en is True:
            key1_en = False
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
            photo_file_name = "/sd/sample/media/photo/photo_" + str(time.ticks_ms()) + ".jpg"
            img.save(photo_file_name)
            photo = image.Image(photo_file_name)
            font.img_draw_string(photo, 5, 5, photo.width(), 16, "Saved to", 16, (255, 0, 0))
            font.img_draw_string(photo, 5, 20, photo.width(), 32, photo_file_name, 16, (255, 0, 0))
            lcd.display(photo)
            time.sleep(2)
            del photo

    lcd.display(img)
    gc.collect()
