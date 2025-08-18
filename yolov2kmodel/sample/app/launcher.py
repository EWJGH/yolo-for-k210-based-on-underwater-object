import sys
sys.path.append("/sd/sample/resource/pymodule/font")
import os
import json
from board import board_info
from fpioa_manager import fm
from maix import GPIO
import time
import lcd
import image
import machine
import math
import font

config = {
    "launcher_app_names": [
        "led",
        "beep",
        "camera",
        "picture",
        "music",
        "video",
        "fft",
        "recoder",
        "imu",
        "scan",
        "trace",
        "face_detecter",
        "face_recognizer",
        "hand_detecter",
        "object_detecter",
        "face_mask_detecter",
        "face_attribute",
        "mnist"
    ],
    "launcher_app_selected": 0
}

try:
    with open("/sd/sample/config.json", "rb") as f:
        config = json.loads(f.read())
except Exception as e:
    with open("/sd/sample/config.json", "w") as f:
        f.write(json.dumps(config))

fm.register(board_info.KEY0, fm.fpioa.GPIOHS0)
fm.register(board_info.KEY1, fm.fpioa.GPIOHS1)
fm.register(board_info.KEY2, fm.fpioa.GPIOHS2)
key0 = GPIO(GPIO.GPIOHS0, GPIO.IN, GPIO.PULL_UP)
key1 = GPIO(GPIO.GPIOHS1, GPIO.IN, GPIO.PULL_UP)
key2 = GPIO(GPIO.GPIOHS2, GPIO.IN, GPIO.PULL_UP)

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
fb.draw_rectangle(0, HEIGHT - 10, WIDTH, 10, (255, 255, 255), 0, True)
fb.draw_rectangle(config["launcher_app_selected"] * math.ceil(WIDTH / len(config["launcher_app_names"])), HEIGHT - 10, WIDTH // len(config["launcher_app_names"]), 10, (34, 139, 200), 0, True)

icon_c = image.Image("/sd/sample/resource/icon/" + config["launcher_app_names"][config["launcher_app_selected"]] + ".bmp")
fb.draw_image(icon_c, (WIDTH - icon_c.width()) // 2, (HEIGHT - icon_c.height()) // 2 - 20)
if config["launcher_app_selected"] != 0:
    icon_p = image.Image("/sd/sample/resource/icon/" + config["launcher_app_names"][config["launcher_app_selected"] - 1] + ".bmp")
else:
    icon_p = image.Image("/sd/sample/resource/icon/" + config["launcher_app_names"][len(config["launcher_app_names"]) - 1] + ".bmp")
fb.draw_image(icon_p, (WIDTH - int(icon_p.width() * 0.7)) // 2 - 105, (HEIGHT - int(icon_p.height() * 0.7)) // 2 - 10, 0.7, 0.7, alpha=120)
if config["launcher_app_selected"] != (len(config["launcher_app_names"]) - 1):
    icon_n = image.Image("/sd/sample/resource/icon/" + config["launcher_app_names"][config["launcher_app_selected"] + 1] + ".bmp")
else:
    icon_n = image.Image("/sd/sample/resource/icon/" + config["launcher_app_names"][0] + ".bmp")
fb.draw_image(icon_n, (WIDTH - int(icon_n.width() * 0.7)) // 2 + 105, (HEIGHT - int(icon_n.height() * 0.7)) // 2 - 10, 0.7, 0.7, alpha=120)

app_name_string = config["launcher_app_names"][config["launcher_app_selected"]].replace("_", " ").upper()
font.img_draw_string(fb, (WIDTH - (16 * len(app_name_string))) // 2, (HEIGHT + int(icon_n.height() * 0.7)) // 2 + 20, fb.width(), 32, app_name_string, 32, (0, 0, 0))

lcd.display(fb)

while True:
    if key0_en is True or key1_en is True or key2_en is True:
        if key0_en is True:
            key0_en = False
            if config["launcher_app_selected"] != (len(config["launcher_app_names"]) - 1):
                config["launcher_app_selected"] = config["launcher_app_selected"] + 1
            else:
                config["launcher_app_selected"] = 0
        if key1_en is True:
            key1_en = False
            with open("/sd/sample/config.json", "rb") as f:
                    os.remove("/sd/sample/config.json")
            with open("/sd/sample/config.json", "w") as f:
                f.write(json.dumps(config))
            try:
                with open("/sd/main.py", "rb") as f:
                    os.remove("/sd/main.py")
            except Exception as e:
                pass
            with open("/sd/sample/app/" + config["launcher_app_names"][config["launcher_app_selected"]] + ".py", "rb") as f:
                code_src = f.read()
            with open("/sd/main.py", "wb") as f:
                f.write(code_src)
            machine.reset()
        if key2_en is True:
            key2_en = False
            if config["launcher_app_selected"] != 0:
                config["launcher_app_selected"] = config["launcher_app_selected"] - 1
            else:
                config["launcher_app_selected"] = len(config["launcher_app_names"]) - 1

        fb.draw_rectangle(0, 0, WIDTH, HEIGHT, (162, 202, 216), 0, True)
        fb.draw_rectangle(0, HEIGHT - 10, WIDTH, 10, (255, 255, 255), 0, True)
        fb.draw_rectangle(config["launcher_app_selected"] * math.ceil(WIDTH / len(config["launcher_app_names"])), HEIGHT - 10, WIDTH // len(config["launcher_app_names"]), 10, (34, 139, 200), 0, True)

        icon_c = image.Image("/sd/sample/resource/icon/" + config["launcher_app_names"][config["launcher_app_selected"]] + ".bmp")
        fb.draw_image(icon_c, (WIDTH - icon_c.width()) // 2, (HEIGHT - icon_c.height()) // 2 - 20)
        if config["launcher_app_selected"] != 0:
            icon_p = image.Image("/sd/sample/resource/icon/" + config["launcher_app_names"][config["launcher_app_selected"] - 1] + ".bmp")
        else:
            icon_p = image.Image("/sd/sample/resource/icon/" + config["launcher_app_names"][len(config["launcher_app_names"]) - 1] + ".bmp")
        fb.draw_image(icon_p, (WIDTH - int(icon_p.width() * 0.7)) // 2 - 105, (HEIGHT - int(icon_p.height() * 0.7)) // 2 - 10, 0.7, 0.7, alpha=120)
        if config["launcher_app_selected"] != (len(config["launcher_app_names"]) - 1):
            icon_n = image.Image("/sd/sample/resource/icon/" + config["launcher_app_names"][config["launcher_app_selected"] + 1] + ".bmp")
        else:
            icon_n = image.Image("/sd/sample/resource/icon/" + config["launcher_app_names"][0] + ".bmp")
        fb.draw_image(icon_n, (WIDTH - int(icon_n.width() * 0.7)) // 2 + 105, (HEIGHT - int(icon_n.height() * 0.7)) // 2 - 10, 0.7, 0.7, alpha=120)

        app_name_string = config["launcher_app_names"][config["launcher_app_selected"]].replace("_", " ").upper()
        font.img_draw_string(fb, (WIDTH - (16 * len(app_name_string))) // 2, (HEIGHT + int(icon_n.height() * 0.7)) // 2 + 20, fb.width(), 32, app_name_string, 32, (0, 0, 0))

        lcd.display(fb)
