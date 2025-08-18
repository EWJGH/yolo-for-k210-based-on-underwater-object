import os
from board import board_info
from fpioa_manager import fm
from maix import GPIO
import time
import lcd
import image
import machine
from maix import I2S
import video

fm.register(board_info.KEY0, fm.fpioa.GPIOHS0)
fm.register(board_info.KEY1, fm.fpioa.GPIOHS1)
fm.register(board_info.KEY2, fm.fpioa.GPIOHS2)
fm.register(board_info.SPK_CTRL, fm.fpioa.GPIO0)
fm.register(board_info.SPK_WS, fm.fpioa.I2S0_WS)
fm.register(board_info.SPK_SCLK, fm.fpioa.I2S0_SCLK)
fm.register(board_info.SPK_SDOUT, fm.fpioa.I2S0_OUT_D0)
key0 = GPIO(GPIO.GPIOHS0, GPIO.IN, GPIO.PULL_UP)
key1 = GPIO(GPIO.GPIOHS1, GPIO.IN, GPIO.PULL_UP)
key2 = GPIO(GPIO.GPIOHS2, GPIO.IN, GPIO.PULL_UP)
spk_ctl = GPIO(GPIO.GPIO0, GPIO.OUT, value=1)

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

videos_list = os.listdir("/sd/sample/media/video")
videos_index = 0
videos_index_prev = videos_index

i2s_dev = I2S(I2S.DEVICE_0)
i2s_dev.channel_config(I2S.CHANNEL_0, I2S.TRANSMITTER, resolution=I2S.RESOLUTION_16_BIT, cycles=I2S.SCLK_CYCLES_32, align_mode=I2S.STANDARD_MODE)
video_player = video.open("/sd/sample/media/video/" + videos_list[videos_index])
video_player.volume(75)

lcd.init()
WIDTH = lcd.width()
HEIGHT = lcd.height()

auto_switch = False
while True:
    if key0_en is True or key1_en is True or key2_en is True:
        if key0_en is True:
            key0_en = False
            if videos_index == len(videos_list) - 1:
                videos_index = 0
            else:
                videos_index = videos_index + 1
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
            if videos_index == 0:
                videos_index = len(videos_list) - 1
            else:
                videos_index = videos_index - 1

    if videos_index_prev == videos_index:
        if video_player.play() == 0:
            auto_switch = True
            if videos_index == len(videos_list) - 1:
                videos_index = 0
            else:
                videos_index = videos_index + 1
    else:
        videos_index_prev = videos_index
        if auto_switch == True:
            auto_switch = False
            video_player.__del__()
            i2s_dev.__deinit__()
            i2s_dev = I2S(I2S.DEVICE_0)
            i2s_dev.channel_config(I2S.CHANNEL_0, I2S.TRANSMITTER, resolution=I2S.RESOLUTION_16_BIT, cycles=I2S.SCLK_CYCLES_32, align_mode=I2S.STANDARD_MODE)
        video_player = video.open("/sd/sample/media/video/" + videos_list[videos_index])
        video_player.volume(75)
