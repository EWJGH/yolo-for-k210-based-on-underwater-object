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
import math
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

code_type = 0
code_type_dict = {
    0: "Normal",
    1: "Barcode",
    2: "Data Matrices",
    3: "QRCode",
    4: "AprilTag"
}

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
            code_type = code_type + 1
            if code_type == len(code_type_dict):
                code_type = 0

    if code_type == 0:
        pass
    elif code_type == 1:
        roi = (img.width() // 2 - 120, img.height() // 2 - 80, 240, 160)
        gray = img.copy(roi).to_grayscale()
        img.draw_rectangle(roi, color=(0, 255, 0))
        barcodes = gray.find_barcodes((0, 0, gray.width(), gray.height()))
        if barcodes:
            font.img_draw_string(img, 10, 30, img.width(), 24, barcodes[0].payload(), 24, (255, 0, 0))
    elif code_type == 2:
        roi = (img.width() // 2 - 100, img.height() // 2 - 100, 200, 200)
        gray = img.copy(roi).to_grayscale()
        img.draw_rectangle(roi, color=(0, 255, 0))
        datamatrices = gray.find_datamatrices((0, 0, gray.width(), gray.height()))
        if datamatrices:
            font.img_draw_string(img, 10, 30, img.width(), 24, datamatrices[0].payload(), 24, (255, 0, 0))
    elif code_type == 3:
        roi = (img.width() // 2 - 100, img.height() // 2 - 100, 200, 200)
        gray = img.copy(roi).to_grayscale()
        img.draw_rectangle(roi, color=(0, 255, 0))
        qrcodes = gray.find_qrcodes((0, 0, gray.width(), gray.height()))
        if qrcodes:
            font.img_draw_string(img, 10, 30, img.width(), 24, qrcodes[0].payload(), 24, (255, 0, 0))
    elif code_type == 4:
        roi = (img.width() // 2 - 100, img.height() // 2 - 100, 200, 200)
        gray = img.copy(roi).to_grayscale()
        img.draw_rectangle(roi, color=(0, 255, 0))
        apriltags = gray.find_apriltags((0, 0, gray.width(), gray.height()), families=image.TAG36H11)
        if apriltags:
            def shot_degrees(axis, y, rotation):
                degrees = (180 * rotation) / math.pi
                font.img_draw_string(img, 10, y, img.width(), 24, "{:s}:{:.0f}".format(axis, degrees), 24, (255, 0, 0))
            font.img_draw_string(img, 10, 30, img.width(), 24, "ID:{:d}".format(apriltags[0].id()), 24, (255, 0, 0))
            shot_degrees("X", 50, apriltags[0].x_rotation())
            shot_degrees("Y", 70, apriltags[0].y_rotation())
            shot_degrees("Z", 90, apriltags[0].z_rotation())
    else:
        code_type = 0
    font.img_draw_string(img, 10, 10, img.width(), 24, code_type_dict[code_type], 24, (255, 0, 0))

    lcd.display(img)
    gc.collect()
