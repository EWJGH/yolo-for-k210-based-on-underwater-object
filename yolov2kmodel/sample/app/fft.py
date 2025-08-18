import os
from board import board_info
from fpioa_manager import fm
from maix import GPIO
import time
import lcd
import image
import machine
from maix import I2S
from maix import FFT

fm.register(board_info.KEY1, fm.fpioa.GPIOHS1)
fm.register(board_info.SPK_CTRL, fm.fpioa.GPIO0)
fm.register(board_info.MIC_WS, fm.fpioa.I2S0_WS)
fm.register(board_info.MIC_SCLK, fm.fpioa.I2S0_SCLK)
fm.register(board_info.MIC_SDIN, fm.fpioa.I2S0_IN_D0)
key1 = GPIO(GPIO.GPIOHS1, GPIO.IN, GPIO.PULL_UP)
spk_ctl = GPIO(GPIO.GPIO0, GPIO.OUT, value=0)

key1_en = False
def key_irq_handler(key):
    global key1
    global key1_en
    time.sleep_ms(20)
    if key is key1 and key.value() == 0:
        key1_en = True

key1.irq(key_irq_handler, GPIO.IRQ_FALLING, GPIO.WAKEUP_NOT_SUPPORT, 7)

SAMPLE_RATE = 38640
SAMPLE_POINTS = 1024
FFT_POINTS = 512
HIST_NUM = 50

i2s_dev = I2S(I2S.DEVICE_0)
i2s_dev.channel_config(I2S.CHANNEL_0, I2S.RECEIVER, align_mode=I2S.STANDARD_MODE)
i2s_dev.set_sample_rate(SAMPLE_RATE)

lcd.init()
WIDTH = lcd.width()
HEIGHT = lcd.height()

fb = image.Image(size=(WIDTH, HEIGHT))
fb.draw_rectangle(0, 0, WIDTH, HEIGHT, (162, 202, 216), 0, True)

lcd.display(fb)

hist_width = int(WIDTH / HIST_NUM)

while True:
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

    data = i2s_dev.record(SAMPLE_RATE)
    res = FFT.run(data.to_bytes(), FFT_POINTS)
    amp = FFT.amplitude(res)
    fb.draw_rectangle(0, 0, WIDTH, HEIGHT, (162, 202, 216), 0, True)
    for hist in range(HIST_NUM):
        if amp[hist] > lcd.height():
            hist_height = lcd.height()
        else:
            hist_height = amp[hist]
        fb.draw_rectangle(hist * hist_width, HEIGHT - hist_height, hist_width, hist_height, (255, 255, 255), 1, True)
    lcd.display(fb)
    del data
    del res
    del amp

