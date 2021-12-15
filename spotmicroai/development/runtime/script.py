#!/home/pi/spotmicroai/venv/bin/python3 -u

import os
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as servo
from pick import pick
import time
from math import pi
import RPi.GPIO as GPIO
# from spot_micro_kinematics_python.spot_micro_stick_figure import SpotMicroStickFigure

from spotmicroai.utilities.log import Logger
from spotmicroai.utilities.config import Config


log = Logger().setup_logger('Powering up SPARKY!')
log.info('setup')


pca=None
pca9685_address = 0x40
pca9685_reference_clock_speed = int(Config().get(
    'motion_controller[*].boards[*].pca9685_1[*].reference_clock_speed | [0] | [0] | [0]'))
pca9685_frequency = int(
    Config().get('motion_controller[*].boards[*].pca9685_1[*].frequency | [0] | [0] | [0]'))


gpio_port = Config().get(Config.ABORT_CONTROLLER_GPIO_PORT)

GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_port, GPIO.OUT)
GPIO.output(gpio_port, False)
time.sleep(1)

i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c_bus=i2c, address=pca9685_address, reference_clock_speed=pca9685_reference_clock_speed)
pca.frequency = pca9685_frequency

# Some References that might be useful for coding motion of servos
#order: 
# rear_left: 8, 9, 10(shoulder, leg, feet), = 0, 1, 2
# rear_right: 12, 13, 14                    = 3, 4, 5
# front_left: 4, 5, 6                       = 6, 7, 8
# front_right: 0, 1, 2                      = 9, 10, 11
# servos_pos_help is meant to only help see where the above servo positions are found and nothing else
# servos_pos_help2 is meant to only help see where the above servo positions are found and nothing else
servos = [(8,90),(9,90),(10,90),(12,90),(13,90),(14,90),(4,90),(5,90),(6,90),(0,90),(1,90),(2,90)]
servo_names = {'rsl':0, 'rll':1, 'rfl':2, 
               'rsr':3, 'rlr':4, 'rfr':5, 
               'fsl':6, 'fll':7, 'ffl':8, 
               'fsr':9, 'flr':10, 'ffr':11}
servo_limits = [(40,140), (30, 150), (20, 160)] * 4
servos_pos_help     = [  8,   9, 10,  12, 13,  14,   4,   5,  6,  0,   1,   2]
servos_pos_help2    = [  0,   1,  2,   3,  4,   5,   6,   7,  8,  9,  10, 11]
rest_angles         = [ 75, 100,  0, 105, 80, 180, 105, 100, 0, 75, 80, 180]
stand_angles        = [ 75,  90, 90, 105, 90,  90, 105, 90, 90, 75, 90,  90]
front_shoulders = [4, 0]
back_shoulders = [8,12]

leg_angs = ()

def set_servo_angle(s, a):
    #s = position of servo in list
    active_servo = servo.Servo(pca.channels[servos[s][0]])
    active_servo.set_pulse_width_range(min_pulse=500, max_pulse=2500)
    active_servo.angle=a
    servos[s] = (servos[s][0], a)
    print("servo: " + list(servo_names.keys())[s] + " | angle = " + str(servos[s][1]))

def check_limits(lowest, highest, cur):
    if cur < lowest:
        return lowest
    elif cur > highest:
        return highest
    else:
        return cur

def turn_servo(servo_name, d):
    #sn is the servo index in the array
    sn = servo_names[servo_name]
    limits = servo_limits[sn]
    if d > 0:
        for i in range(d):
            set_servo_angle(sn, check_limits(limits[0], limits[1], servos[sn][1]+1))
            time.sleep(0.01)
    else:
        for i in range(abs(d)):
            set_servo_angle(sn, check_limits(limits[0], limits[1], servos[sn][1]-1))
            time.sleep(0.01)
 

def set_leg_angs():
    angs = ((servos[3][1],servos[4][1],servos[5][1]), 
                (servos[9][1],servos[10][1],servos[11][1]),
                (servos[6][1],servos[7][1],servos[8][1]),
                (servos[0][1],servos[1][1],servos[2][1]))
    leg_angs = angs
    return angs


def test_servo_limits():
    for i in range(len(servo_limits)):
        for n in range(servo_limits[i][0], servo_limits[i][1]):
            set_servo_angle(servos[i][0], n)
            time.sleep(.1)
        time.sleep(1)
        for n in range(servo_limits[i][1], servo_limits[i][0]):
            set_servo_angle(servos[i][0], n)
            time.sleep(.1)
        time.sleep(2)

def squat(a):
    set_servo_angle(1, servos[1][1]+a)
    set_servo_angle(2, servos[2][1]-a)
    set_servo_angle(7, servos[7][1]+a)
    set_servo_angle(8, servos[8][1]-a)
    set_servo_angle(4, servos[4][1]-a)
    set_servo_angle(5, servos[5][1]+a)
    set_servo_angle(10, servos[10][1]-a)
    set_servo_angle(11, servos[11][1]+a)

def unsquat(a):
    set_servo_angle(1, servos[1][1]-a)
    set_servo_angle(2, servos[2][1]+a)
    set_servo_angle(7, servos[7][1]-a)
    set_servo_angle(8, servos[8][1]+a)
    set_servo_angle(4, servos[4][1]+a)
    set_servo_angle(5, servos[5][1]-a)
    set_servo_angle(10, servos[10][1]+a)
    set_servo_angle(11, servos[11][1]-a)

def crouch(a):
    set_servo_angle(1, servos[1][1]+a)
    set_servo_angle(2, servos[2][1]-(a*2))
    set_servo_angle(7, servos[7][1]+a)
    set_servo_angle(8, servos[8][1]-(a*2))
    set_servo_angle(4, servos[4][1]-a)
    set_servo_angle(5, servos[5][1]+(a*2))
    set_servo_angle(10, servos[10][1]-a)
    set_servo_angle(11, servos[11][1]+(a*2))

def uncrouch(a):
    set_servo_angle(1, servos[1][1]-a)
    set_servo_angle(2, servos[2][1]+(a*2))
    set_servo_angle(7, servos[7][1]-a)
    set_servo_angle(8, servos[8][1]+(a*2))
    set_servo_angle(4, servos[4][1]+a)
    set_servo_angle(5, servos[5][1]-(a*2))
    set_servo_angle(10, servos[10][1]+a)
    set_servo_angle(11, servos[11][1]-(a*2))
    
def rest_position():
    for x in range(len(servos)):
        set_servo_angle(x, rest_angles[x])
    time.sleep(0.1)

def sit_back(a):
    set_servo_angle(1, servos[1][1]-a)
    set_servo_angle(2, servos[2][1]-(0.5*a))
    set_servo_angle(4, servos[4][1]+a)
    set_servo_angle(5, servos[5][1]+(0.5*a))

def stand_straight():
    for x in range(len(servos)):
        set_servo_angle(x, stand_angles[x])
    time.sleep(0.1)

def roll_left(a):
    set_servo_angle(6, servos[6][1]+a)
    set_servo_angle(9, servos[9][1]+a)
    set_servo_angle(0, servos[0][1]-a)
    set_servo_angle(3, servos[3][1]-a)

def roll_right(a):
    set_servo_angle(6, servos[6][1]-a)
    set_servo_angle(9, servos[9][1]-a)
    set_servo_angle(0, servos[0][1]+a)
    set_servo_angle(3, servos[3][1]+a)

def set_body(sf):
    sf.set_body_angles(theta=10*pi/180)

def loop_tests():
    while(True):
        action= input("What would you like to do? ")
        if action=="sit":
            rest_position()
        elif action=="stand":
            stand_straight()
        elif action=="sit back":
            try:
                while(True):
                    sit_back(.2)
                    time.sleep(.01)
            except KeyboardInterrupt:
                print("stopped")
                pass
        elif action=="roll left":
            try:
                while(True):
                    roll_left(.2)
                    time.sleep(.01)
            except KeyboardInterrupt:
                print("stopped")
                pass
        elif action=="roll right":
            try:
                while(True):
                    roll_right(.2)
                    time.sleep(.01)
            except KeyboardInterrupt:
                print("stopped")
                pass
        elif action=="squat":
            while(True):
                itrpt = input("type stop to STOP: ")
                if itrpt=="stop":
                    break
                else:
                    squat(5)
                    time.sleep(.1)
        elif action=="unsquat":
            while(True):
                itrpt = input("type stop to STOP: ")
                if itrpt=="stop":
                    break
                else:
                    unsquat(5)
                    time.sleep(.1)
        elif action=="crouch":
            while(True):
                itrpt = input("type stop to STOP: ")
                if itrpt=="stop":
                    break
                else:
                    crouch(2)
                    time.sleep(.1)
        elif action=="uncrouch":
            while(True):
                itrpt = input("type stop to STOP: ")
                if itrpt=="stop":
                    break
                else:
                    uncrouch(2)
                    time.sleep(.1)
        elif action=="test servos":
            try:
                while(True):
                    try:
                        part = input("which part would you like to test? ")
                        angle = int(input("by how many degrees? "))
                        print(angle)
                    except:
                        break
                    try:
                        turn_servo(part, angle)
                    except:
                        print("That didn't work....")
                        # if part=="fsl":
                        #     turn_fsl(angle)
                        # elif part=="fsr":
                        #     turn_fsr(angle)
                        # elif part=="fll":
                        #     turn_fll(angle)
                        # elif part=="flr":
                        #     turn_flr(angle)
                        # elif part=="ffl":
                        #     turn_ffl(angle)
                        # elif part=="ffr":
                        #     turn_ffr(angle)
            except KeyboardInterrupt:
                print("exiting servo test...")
                pass
        elif action=="test limits":
            try:
                while(True):
                    input("Press anything to begin, >>>")
                    test_servo_limits()
            except KeyboardInterrupt():
                print("<<<")
                pass
        elif action=="quit" or action=="exit":
            print("terminating...")
            break
        else:
            print("terminating...")
            break

def pos_1():
    stand_straight()
    turn

if __name__=="__main__":
    #sparky = SpotMicroStickFigure()

    # coordinates = sparky.get_leg_coordinates()
    # for c in coordinates:
    #     for cord in c:
    #         print(c)
    # print()
    # print("Initial coordinates: ")
    # print(coordinates)

    # angles = sparky.get_leg_angles()
    # print()
    # print("Initial angles: ")
    # print(angles)
    # print()

    stand_straight()
    #time.sleep(5)


    # Iterate over the joystick devices.
    print('Available devices:')

    for fn in os.listdir('/dev/input'):
        if fn.startswith('js'):
            log.info(('  /dev/input/%s' % (fn)))

    # We'll store the states here.
    axis_states = {}
    button_states = {}

    # These constants were borrowed from linux/input.h
    axis_names = {
        0x00: 'x',
        0x01: 'y',
        0x02: 'z',
        0x03: 'rx',
        0x04: 'ry',
        0x05: 'rz',
        0x06: 'trottle',
        0x07: 'rudder',
        0x08: 'wheel',
        0x09: 'gas',
        0x0a: 'brake',
        0x10: 'hat0x',
        0x11: 'hat0y',
        0x12: 'hat1x',
        0x13: 'hat1y',
        0x14: 'hat2x',
        0x15: 'hat2y',
        0x16: 'hat3x',
        0x17: 'hat3y',
        0x18: 'pressure',
        0x19: 'distance',
        0x1a: 'tilt_x',
        0x1b: 'tilt_y',
        0x1c: 'tool_width',
        0x20: 'volume',
        0x28: 'misc',
    }

    button_names = {
        0x120: 'trigger',
        0x121: 'thumb',
        0x122: 'thumb2',
        0x123: 'top',
        0x124: 'top2',
        0x125: 'pinkie',
        0x126: 'base',
        0x127: 'base2',
        0x128: 'base3',
        0x129: 'base4',
        0x12a: 'base5',
        0x12b: 'base6',
        0x12f: 'dead',
        0x130: 'X',
        0x131: 'O',
        0x132: 'c',
        0x133: 'triangle',
        0x134: 'square',
        0x135: 'z',
        0x136: 'tl',
        0x137: 'tr',
        0x138: 'tl2',
        0x139: 'tr2',
        0x13a: 'select',
        0x13b: 'start',
        0x13c: 'mode',
        0x13d: 'thumbl',
        0x13e: 'thumbr',

        0x220: 'dpad_up',
        0x221: 'dpad_down',
        0x222: 'dpad_left',
        0x223: 'dpad_right',

        # XBox 360 controller uses these codes.
        0x2c0: 'dpad_left',
        0x2c1: 'dpad_right',
        0x2c2: 'dpad_up',
        0x2c3: 'dpad_down',
    }

    axis_map = []
    button_map = []

    try:
        print("Connecting to Bluetooth Controller...")

        # Open the joystick device.
        connected_device = Config().get('remote_controller_controller[0].remote_controller[0].device')
        fn = '/dev/input/' + str(connected_device)
        print(('Opening %s...' % fn))
        jsdev = open(fn, 'rb')

        # Get the device name.
        # buf = bytearray(63)
        buf = array.array('B', [0] * 64)
        ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)
        js_name = buf.tostring().rstrip(b'\x00').decode('utf-8')
        print(('Device name: %s' % js_name))

        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(jsdev, 0x80016a11, buf)  # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array('B', [0])
        ioctl(jsdev, 0x80016a12, buf)  # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(jsdev, 0x80406a32, buf)  # JSIOCGAXMAP

        for axis in buf[:num_axes]:
            axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
            axis_map.append(axis_name)
            axis_states[axis_name] = 0.0

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(jsdev, 0x80406a34, buf)  # JSIOCGBTNMAP

        for btn in buf[:num_buttons]:
            btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
            button_map.append(btn_name)
            button_states[btn_name] = 0

        print(('%d axes found: %s' % (num_axes, ', '.join(axis_map))))
        print(('%d buttons found: %s' % (num_buttons, ', '.join(button_map))))
    except:
        print("< ERROR >  |  Coudn't connect to Bluetooth Device")
        pass
    

    # print()
    # print(sparky.x)
    # print(sparky.y)
    # print(sparky.z)
    # print(sparky.ht_body)
    # print()

    # sparky.set_absolute_body_pose(ht_body)

    # Main event loop
    while True:
        try: 
            jsdev, evbuf
            print("If controller is connected, then: ")
            evbuf = jsdev.read(8)
            if evbuf:
                time_data, value, type, number = struct.unpack('IhBB', evbuf)

                if type & 0x80:
                    print("(initial)")

                if type & 0x01:
                    button = button_map[number]
                    if button:
                        button_states[button] = value
                        if button=='X' and value:
                            loop_tests()
                        elif value:
                            print(("%s pressed" % (button)))
                        else:
                            print(("%s released" % (button)))

                if type & 0x02:
                    axis = axis_map[number]
                    if axis:
                        fvalue = value / 32767.0
                        axis_states[axis] = fvalue
                        print(("%s: %.3f" % (axis, fvalue)))
        except: 
            print("Couldn't connect to bluetooth.")
            print("Resuming with keyboard command test...")
            loop_tests()


        
    # while(True):
    #     roll_left(10)
    #     time.sleep(2)
    #     roll_right(10)
    #     time.sleep(2)
    #     roll_right(10)
    #     time.sleep(2)
    #     roll_left(10)
    #     time.sleep(2)
    # try:
    #     set_body()
    #     while(True):
    #         rest_position()
    #         print(sparky.get_leg_angles())
    #         time.sleep(5)
    #         stand_straight()
    #         print(sparky.get_leg_angles())
    #         time.sleep(5)
    #         roll_left()
    #         time.sleep(5)
    # except:
    #     log.error("trouble connecting to the servos")


