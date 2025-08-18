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
from maix import KPU
import font

fm.register(board_info.KEY1, fm.fpioa.GPIOHS1)
key1 = GPIO(GPIO.GPIOHS1, GPIO.IN, GPIO.PULL_UP)

key1_en = False
def key_irq_handler(key):
    global key1
    global key1_en
    time.sleep_ms(20)
    if key is key1 and key.value() == 0:
        key1_en = True

key1.irq(key_irq_handler, GPIO.IRQ_FALLING, GPIO.WAKEUP_NOT_SUPPORT, 7)

sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
sensor.set_vflip(True)

resize_img = image.Image(size=(320, 256))
anchor = (0.8125, 0.4556, 1.1328, 1.2667, 1.8594, 1.4889, 1.4844, 2.2000, 2.6484, 2.9333)
names = ['hand']

hand_detecter = KPU()
hand_detecter.load_kmodel("/sd/sample/resource/kmodel/hand_detect.kmodel")
hand_detecter.init_yolo2(anchor, anchor_num=len(anchor) // 2, img_w=320, img_h=240, net_w=320, net_h=256, layer_w=10, layer_h=8, threshold=0.7, nms_value=0.3, classes=len(names))

lcd.init()
WIDTH = lcd.width()
HEIGHT = lcd.height()

while True:
    img = sensor.snapshot()

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

    resize_img.draw_image(img, 0, 0).pix_to_ai()
    hand_detecter.run_with_output(resize_img)
    hands = hand_detecter.regionlayer_yolo2()
    for hand in hands:
        img.draw_rectangle(hand[0], hand[1], hand[2], hand[3], color=(0, 255, 0))
        font.img_draw_string(img, hand[0] + 2, hand[1] + 2, img.width(), 16, "%.2f" % (hand[5]), 16, (0, 255, 0))
        font.img_draw_string(img, hand[0] + 2, hand[1] + 16, img.width(), 16, names[hand[4]], 16, (0, 255, 0))

    lcd.display(img)
    gc.collect()
