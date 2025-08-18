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

anchor = (0.1075, 0.126875, 0.126875, 0.175, 0.1465625, 0.2246875, 0.1953125, 0.25375, 0.2440625, 0.351875, 0.341875, 0.4721875, 0.5078125, 0.6696875, 0.8984375, 1.099687, 2.129062, 2.425937)
names = ['face']

face_detecter = KPU()
face_detecter.load_kmodel("/sd/sample/resource/kmodel/face_detect_320x240.kmodel")
face_detecter.init_yolo2(anchor, anchor_num=len(anchor) // 2, img_w=320, img_h=240, net_w=320, net_h=240, layer_w=10, layer_h=8, threshold=0.5, nms_value=0.2, classes=len(names))

ld5_kpu = KPU()
ld5_kpu.load_kmodel("/sd/sample/resource/kmodel/ld5.kmodel")

pos_face_attr = ["Male ", "Mouth Open ", "Smiling ", "Glasses"]
neg_face_attr = ["Female ", "Mouth Closed", "No Smile", "No Glasses"]

fac_kpu = KPU()
fac_kpu.load_kmodel("/sd/sample/resource/kmodel/fac.kmodel")

def extend_box(x, y, w, h, scale):
    x1 = int(x - scale * w)
    x2 = int(x + w - 1 + scale * w)
    y1 = int(y - scale * h)
    y2 = int(y + h - 1 + scale * h)
    x1 = x1 if x1 > 0 else 0
    x2 = x2 if x2 < (320 - 1) else (320 - 1)
    y1 = y1 if y1 > 0 else 0
    y2 = y2 if y2 < (240 - 1) else (240 - 1)
    return x1, y1, x2 - x1 + 1, y2 - y1 + 1

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

    face_detecter.run_with_output(img)
    faces = face_detecter.regionlayer_yolo2()
    for face in faces:
        x, y, w, h = extend_box(face[0], face[1], face[2], face[3], 0.08)
        img.draw_rectangle(x, y, w, h, color=(0, 255, 0))
        face_img = img.cut(x, y, w, h)
        resize_img = face_img.resize(128, 128)
        resize_img.pix_to_ai()
        output = ld5_kpu.run_with_output(resize_img, getlist=True)
        for i in range(len(output) // 2):
            point_x = int(KPU.sigmoid(output[2 * i]) * w + x)
            point_y = int(KPU.sigmoid(output[2 * i + 1]) * h + y)
            img.draw_cross(point_x, point_y, size=5, color=(0, 0, 255))
        output = fac_kpu.run_with_output(resize_img, getlist=True)
        for i in range(len(output)):
            if KPU.sigmoid(output[i]) > 0.5:
                font.img_draw_string(img, x + w + 2, y + i * 16 + 2, img.width(), 16, "%s" % (pos_face_attr[i]), 16, (255, 0, 0))
            else:
                font.img_draw_string(img, x + w + 2, y + i * 16 + 2, img.width(), 16, "%s" % (neg_face_attr[i]), 16, (0, 0, 255))
        del face_img
        del resize_img

    lcd.display(img)
    gc.collect()
