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
from maix import I2S
import audio
import font

fm.register(board_info.KEY1, fm.fpioa.GPIOHS1)
fm.register(board_info.KEY2, fm.fpioa.GPIOHS2)
fm.register(board_info.SPK_CTRL, fm.fpioa.GPIO0)
fm.register(board_info.MIC_WS, fm.fpioa.I2S0_WS)
fm.register(board_info.MIC_SCLK, fm.fpioa.I2S0_SCLK)
fm.register(board_info.MIC_SDIN, fm.fpioa.I2S0_IN_D0)
key1 = GPIO(GPIO.GPIOHS1, GPIO.IN, GPIO.PULL_UP)
key2 = GPIO(GPIO.GPIOHS2, GPIO.IN, GPIO.PULL_UP)
spk_ctl = GPIO(GPIO.GPIO0, GPIO.OUT, value=0)

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

SAMPLE_RATE = 16000
SINGLE_SAMPLE_POINTS = 2048
WAV_CHANNEL = 2

i2s_dev = I2S(I2S.DEVICE_0)
i2s_dev.channel_config(I2S.CHANNEL_0, I2S.RECEIVER, align_mode=I2S.STANDARD_MODE)
i2s_dev.set_sample_rate(SAMPLE_RATE)

try:
    os.mkdir("/sd/sample/media/recode")
except Exception:
    pass

lcd.init()
WIDTH = lcd.width()
HEIGHT = lcd.height()

fb = image.Image(size=(WIDTH, HEIGHT))
fb.draw_rectangle(0, 0, WIDTH, HEIGHT, (162, 202, 216), 0, True)

pic_stop = image.Image("/sd/sample/resource/picture/recoder/recoder_stop.bmp")
pic_start = image.Image("/sd/sample/resource/picture/recoder/recoder_start.bmp")
fb.draw_image(pic_start, ((WIDTH - pic_stop.width()) // 2) - 10, (HEIGHT // 2) - (pic_stop.height() // 2))

lcd.display(fb)

start_recode = False
while True:
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
            start_recode = not start_recode

        fb.draw_rectangle(0, 0, WIDTH, HEIGHT, (162, 202, 216), 0, True)

        if start_recode is True:
            fb.draw_image(pic_stop, (WIDTH - pic_stop.width()) // 2, (HEIGHT // 2) - (pic_stop.height() // 2))
            recode_file_name = "/sd/sample/media/recode/recode_" + str(time.ticks_ms()) + ".wav"
            audio_recorder = audio.Audio(path=recode_file_name, is_create=True, samplerate=SAMPLE_RATE)
            font.img_draw_string(fb, 5, 5, fb.width(), 16, "Saving to", 16, (255, 0, 0))
            font.img_draw_string(fb, 5, 20, fb.width(), 32, recode_file_name, 16, (255, 0, 0))
            datas = []
        else:
            fb.draw_image(pic_start, ((WIDTH - pic_stop.width()) // 2) - 10, (HEIGHT // 2) - (pic_stop.height() // 2))
            audio_recorder.finish()

        lcd.display(fb)

    if start_recode is True:
        data = i2s_dev.record(WAV_CHANNEL * SINGLE_SAMPLE_POINTS)
        if len(datas) is not 0:
            audio_recorder.record(datas[0])
            datas.pop(0)
        i2s_dev.wait_record()
        datas.append(data)
