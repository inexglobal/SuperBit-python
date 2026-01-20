# Your new file!
from microbit import *
import neopixel
import music

class SuperBit:
    def __init__(self):
        # RGB LED (Neopixel บน pin12)
        self.yahStrip = neopixel.NeoPixel(pin12, 4)

        # Address ของ PCA9685
        self.PCA9685_ADDR = 0x40

        # Register ของ PCA9685
        self.MODE1     = 0x00
        self.PRESCALE  = 0xFE
        self.LED0_ON_L = 0x06

        # สถานะการ init
        self.initialized = False

        # ค่าคงที่สำหรับ Stepper
        self.STP_CHA_L = 0
        self.STP_CHA_H = 2048
        self.STP_CHB_L = 0
        self.STP_CHB_H = 2048
        self.STP_CHC_L = 0
        self.STP_CHC_H = 2048
        self.STP_CHD_L = 0
        self.STP_CHD_H = 2048

        # index ของ stepper motor
        self.B1 = 1
        self.B2 = 2

        # init i2c
        i2c.init()

    # -------------------------------
    # I2C helper
    # -------------------------------
    def i2c_write(self, addr, reg, value):
        i2c.write(addr, bytes([reg, value]))

    def i2c_cmd(self, addr, value):
        i2c.write(addr, bytes([value]))

    def i2c_read(self, addr, reg):
        i2c.write(addr, bytes([reg]))
        val = i2c.read(addr, 1)
        return val[0]

    # -------------------------------
    # PCA9685
    # -------------------------------
    def init_pca9685(self):
        self.i2c_write(self.PCA9685_ADDR, self.MODE1, 0x00)
        self.set_freq(50)
        self.initialized = True

    def set_freq(self, freq):
        prescaleval = 25000000.0
        prescaleval /= 4096.0
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = int(prescaleval + 0.5)

        oldmode = self.i2c_read(self.PCA9685_ADDR, self.MODE1)
        newmode = (oldmode & 0x7F) | 0x10
        self.i2c_write(self.PCA9685_ADDR, self.MODE1, newmode)
        self.i2c_write(self.PCA9685_ADDR, self.PRESCALE, prescale)
        self.i2c_write(self.PCA9685_ADDR, self.MODE1, oldmode)
        sleep(5)
        self.i2c_write(self.PCA9685_ADDR, self.MODE1, oldmode | 0xA1)

    def set_pwm(self, channel, on, off):
        if channel < 0 or channel > 15:
            return
        if not self.initialized:
            self.init_pca9685()

        reg = self.LED0_ON_L + 4 * channel
        data = bytes([
            reg,
            on & 0xFF,
            (on >> 8) & 0xFF,
            off & 0xFF,
            (off >> 8) & 0xFF
        ])
        i2c.write(self.PCA9685_ADDR, data)

    # -------------------------------
    # Stepper
    # -------------------------------
    def set_stepper(self, index, direction):
        if index == self.B1:
            if direction:
                self.set_pwm(11, self.STP_CHA_L, self.STP_CHA_H)
                self.set_pwm(9,  self.STP_CHB_L, self.STP_CHB_H)
                self.set_pwm(10, self.STP_CHC_L, self.STP_CHC_H)
                self.set_pwm(8,  self.STP_CHD_L, self.STP_CHD_H)
            else:
                self.set_pwm(8,  self.STP_CHA_L, self.STP_CHA_H)
                self.set_pwm(10, self.STP_CHB_L, self.STP_CHB_H)
                self.set_pwm(9,  self.STP_CHC_L, self.STP_CHC_H)
                self.set_pwm(11, self.STP_CHD_L, self.STP_CHD_H)
        else:
            if direction:
                self.set_pwm(12, self.STP_CHA_L, self.STP_CHA_H)
                self.set_pwm(14, self.STP_CHB_L, self.STP_CHB_H)
                self.set_pwm(13, self.STP_CHC_L, self.STP_CHC_H)
                self.set_pwm(15, self.STP_CHD_L, self.STP_CHD_H)
            else:
                self.set_pwm(15, self.STP_CHA_L, self.STP_CHA_H)
                self.set_pwm(13, self.STP_CHB_L, self.STP_CHB_H)
                self.set_pwm(14, self.STP_CHC_L, self.STP_CHC_H)
                self.set_pwm(12, self.STP_CHD_L, self.STP_CHD_H)

    def stop_motor(self, index):
        self.set_pwm(index, 0, 0)
        self.set_pwm(index + 1, 0, 0)

    # -------------------------------
    # Motor
    # -------------------------------
    def motor_run(self, index, speed):
        if not self.initialized:
            self.init_pca9685()
            self.initialized = True

        speed = speed * 16
        if speed >= 4096:
            speed = 4095
        if speed <= -4096:
            speed = -4095

        a = index
        b = index + 1

        if a > 10:
            if speed >= 0:
                self.set_pwm(a, 0, speed)
                self.set_pwm(b, 0, 0)
            else:
                self.set_pwm(a, 0, 0)
                self.set_pwm(b, 0, -speed)
        else:
            if speed >= 0:
                self.set_pwm(b, 0, speed)
                self.set_pwm(a, 0, 0)
            else:
                self.set_pwm(b, 0, 0)
                self.set_pwm(a, 0, -speed)

    def motor_run_dual(self, motor1, speed1, motor2, speed2):
        self.motor_run(motor1, speed1)
        self.motor_run(motor2, speed2)

    # -------------------------------
    # RGB LED
    # -------------------------------
    def rgb_program(self):
        return self.yahStrip

    # -------------------------------
    # Music
    # -------------------------------
    def play_music(self, index):
        melodies = {
            "dadadum": music.DADADADUM,
            "birthday": music.BIRTHDAY,
            "entertainer": music.ENTERTAINER,
            "prelude": music.PRELUDE,
            "ode": music.ODE,
            "nyan": music.NYAN,
            "ringtone": music.RINGTONE,
            "funk": music.FUNK,
            "blues": music.BLUES,
            "wedding": music.WEDDING,
            "funeral": music.FUNERAL,
            "punchline": music.PUNCHLINE,
            "baddy": music.BADDY,
            "chase": music.CHASE,
            "ba_ding": music.BA_DING,
            "wawawawaa": music.WAWAWAWAA,
            "jump_up": music.JUMP_UP,
            "jump_down": music.JUMP_DOWN,
            "power_up": music.POWER_UP,
            "power_down": music.POWER_DOWN
        }
        if index in melodies:
            music.play(melodies[index])

    # -------------------------------
    # Servo
    # -------------------------------
    def servo(self, num, value):
        us = (value * 1800 / 180 + 600)
        pwm = int(us * 4096 / 20000)
        self.set_pwm(num, 0, pwm)

    def servo2(self, num, value):
        newvalue = int(value * 180 / 270)
        us = (newvalue * 1800 / 180 + 600)
        pwm = int(us * 4096 / 20000)
        self.set_pwm(num, 0, pwm)

    def servo3(self, num, pos, value):
        if pos == "stop":
            us = (86 * 1800 / 180 + 600)
            pwm = int(us * 4096 / 20000)
            self.set_pwm(num, 0, pwm)
        elif pos == "forward":
            us = ((90 - value) * 1800 / 180 + 600)
            pwm = int(us * 4096 / 20000)
            self.set_pwm(num, 0, pwm)
        elif pos == "reverse":
            us = ((90 + value) * 1800)
