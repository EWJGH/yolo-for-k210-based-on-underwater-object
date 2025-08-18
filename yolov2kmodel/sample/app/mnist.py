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

mnist_recognizer = KPU()
mnist_recognizer.load_kmodel("/sd/sample/resource/kmodel/uint8_mnist_cnn_model.kmodel")

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

    roi = ((img.width() - img.height()) // 2 - 1, 0, img.height(), img.height())
    img.draw_rectangle(roi, color=(0, 255, 0))
    gray_img = img.copy(roi).to_grayscale().resize(112, 112)
    gray_img.invert()
    gray_img.strech_char(1)
    gray_img.pix_to_ai()
    output = mnist_recognizer.run_with_output(gray_img, getlist=True)
    number = output.index(max(output))
    score = KPU.sigmoid(max(output))
    font.img_draw_string(img, 2, 2, img.width(), 16, str(number), 16, (255, 0, 0))
    font.img_draw_string(img, 2, 16 + 2, img.width(), 16, str(score), 16, (255, 0, 0))
    del gray_img

    lcd.display(img)
    gc.collect()
