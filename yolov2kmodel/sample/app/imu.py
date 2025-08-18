import sys
sys.path.append("/sd/sample/resource/pymodule/imu")
import os
from board import board_info
from fpioa_manager import fm
from maix import GPIO
import time
import lcd
import image
import machine
from machine import I2C
from sh3001 import SH3001
from machine import Timer
import math

fm.register(board_info.KEY1, fm.fpioa.GPIOHS1)
fm.register(board_info.IMU_SCL, fm.fpioa.I2C0_SCLK)
fm.register(board_info.IMU_SDA, fm.fpioa.I2C0_SDA)
key1 = GPIO(GPIO.GPIOHS1, GPIO.IN, GPIO.PULL_UP)

key1_en = False
def key_irq_handler(key):
    global key1
    global key1_en
    time.sleep_ms(20)
    if key is key1 and key.value() == 0:
        key1_en = True

key1.irq(key_irq_handler, GPIO.IRQ_FALLING, GPIO.WAKEUP_NOT_SUPPORT, 7)

i2c = I2C(I2C.I2C0, scl=board_info.IMU_SCL, sda=board_info.IMU_SDA)

sh3001 = SH3001(i2c, attitude=True)

pitch = 0.0
roll = 0.0
yaw = 0.0

def timer_timeout_cb(timer):
    global pitch
    global roll
    global yaw
    pitch, roll, yaw = sh3001.get_attitude()

timer0 = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC, period=490, unit=Timer.UNIT_US, callback=timer_timeout_cb)

cube_points = [[-30.0, -30.0, -30.0], [-30.0, 30.0, -30.0], [30.0, 30.0, -30.0], [30.0, -30.0, -30.0], [-10.0, -10.0, 30.0], [-10.0, 10.0, 30.0], [10.0, 10.0, 30.0], [10.0, -10.0, 30.0]]
cube_points_dest = [[0.0 for axis_index in range(len(cube_points[0]))] for point_index in range(len(cube_points))]
cube_lines_points_ids = [[1, 2], [2, 3], [3, 4], [4, 1], [5, 6], [6, 7], [7, 8], [8, 5], [8, 4], [7, 3], [6, 2], [5, 1]]

def angle_to_radian(angle):
    return (angle * (math.pi / 180))

lcd.init()
WIDTH = lcd.width()
HEIGHT = lcd.height()

fb = image.Image(size=(WIDTH, HEIGHT))

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

    timer0.stop()

    rx =    [[1, 0, 0],
            [0, math.cos(angle_to_radian(pitch)), -math.sin(angle_to_radian(pitch))],
            [0, math.sin(angle_to_radian(pitch)), math.cos(angle_to_radian(pitch))]]
    ry =    [[math.cos(angle_to_radian(roll)), 0, math.sin(angle_to_radian(roll))],
            [0, 1, 0],
            [-math.sin(angle_to_radian(roll)), 0, math.cos(angle_to_radian(roll))]]
    rz =    [[math.cos(angle_to_radian(yaw)), -math.sin(angle_to_radian(yaw)), 0],
            [math.sin(angle_to_radian(yaw)), math.cos(angle_to_radian(yaw)), 0],
            [0, 0, 1]]

    p = [0.0 for axis_index in range(len(cube_points[0]))]

    for point_index in range(len(cube_points)):
        for axis_index in range(len(cube_points[0])):
            p[axis_index] = cube_points[point_index][0] * rx[axis_index][0] + cube_points[point_index][1] * rx[axis_index][1] + cube_points[point_index][2] * rx[axis_index][2]
        for axis_index in range(len(cube_points[0])):
            p[axis_index] = p[0] * ry[axis_index][0] + p[1] * ry[axis_index][1] + p[2] * ry[axis_index][2]
        for axis_index in range(len(cube_points[0])):
            cube_points_dest[point_index][axis_index] = p[0] * rz[axis_index][0] + p[1] * rz[axis_index][1] + p[2] * rz[axis_index][2]

    fb.draw_rectangle((fb.width() // 2) - 52 - 1, (fb.height() // 2) - 52 - 1, (fb.width() // 2) + 52 - 1, (fb.height() // 2) + 52 - 1, (0, 0, 0), 0, True)

    for line_index in range(len(cube_lines_points_ids)):
        fb.draw_line(int((fb.width() // 2) + cube_points_dest[cube_lines_points_ids[line_index][0] - 1][0]),
                    int((fb.height() // 2) + cube_points_dest[cube_lines_points_ids[line_index][0] - 1][1]),
                    int((fb.width() // 2) + cube_points_dest[cube_lines_points_ids[line_index][1] - 1][0]),
                    int((fb.height() // 2) + cube_points_dest[cube_lines_points_ids[line_index][1] - 1][1]),
                    (255, 255, 255))
    fb.draw_line(int((fb.width() // 2) + cube_points_dest[1 - 1][0]),
                  int((fb.height() // 2) + cube_points_dest[1 - 1][1]),
                  int((fb.width() // 2) + cube_points_dest[2 - 1][0]),
                  int((fb.height() // 2) + cube_points_dest[2 - 1][1]),
                  (255, 0, 0), 2);
    fb.draw_line(int((fb.width() // 2) + cube_points_dest[2 - 1][0]),
                  int((fb.height() // 2) + cube_points_dest[2 - 1][1]),
                  int((fb.width() // 2) + cube_points_dest[3 - 1][0]),
                  int((fb.height() // 2) + cube_points_dest[3 - 1][1]),
                  (255, 0, 0), 2);
    fb.draw_line(int((fb.width() // 2) + cube_points_dest[3 - 1][0]),
                  int((fb.height() // 2) + cube_points_dest[3 - 1][1]),
                  int((fb.width() // 2) + cube_points_dest[4 - 1][0]),
                  int((fb.height() // 2) + cube_points_dest[4 - 1][1]),
                  (255, 0, 0), 2);
    fb.draw_line(int((fb.width() // 2) + cube_points_dest[4 - 1][0]),
                  int((fb.height() // 2) + cube_points_dest[4 - 1][1]),
                  int((fb.width() // 2) + cube_points_dest[1 - 1][0]),
                  int((fb.height() // 2) + cube_points_dest[1 - 1][1]),
                  (255, 0, 0), 2);

    timer0.start()

    lcd.display(fb)
