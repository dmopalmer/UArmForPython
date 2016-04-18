from __future__ import division
from pyfirmata import Arduino, util
import math
import time

# PUMP INFORMATION
PUMP_EN = 6
VALVE_EN = 5

# UARM SPECIFICATIONS
MATH_PI = 3.141592653589793238463
MATH_TRANS = 57.2958
MATH_L1 = (10.645 + 0.6)
MATH_L2 = 2.117
MATH_L3 = 14.825
MATH_L4 = 16.02
MATH_L43 = MATH_L4 / MATH_L3

# UARM OFFSETS
TopOffset = -1.5
BottomOffset = 1.5


class Servo(object):
    kAddrOffset = 90
    kAddrAandB = 60

    def __init__(self, board, servonumber, outpin, feedbackpin, constraint=list([0.0, 180.0])):
        self.servonumber = servonumber
        self.board = board
        self.outpin = outpin
        self.feedbackpin = feedbackpin
        self.connected = False
        self.setConstraint(constraint)
        self.offset = self.readServoOffset()
        self.data_a, self.data_b = self.readAandB()

    def attach(self, preserveAngle=True):
        if preserveAngle:
            currangle = self.readAngle()
        self.connection = self.board.get_pin('d:{}:s'.format(self.outpin))
        if preserveAngle:
            self.writeAngle(currangle)
        self.connected = True

    def detach(self):
        self.board.servoDetach(self.outpin)
        self.connected = False

    def readAngle(self, raw=False):
        return self.readToAngle(self.readAnalog(), raw=raw)

    def readAnalog(self, clears=2, samples=1):
        """
        Read the analog value of the servo feedback
        :param clears: number of times to read and discard
        :param samples: number of times to read and average
        :return: int(analog value)
        """
        for _ in range(clears):
            self.board.readAnalogPin(self.feedbackpin)
        sampvalues = [self.board.readAnalogPin(self.feedbackpin)
                      for _ in range(samples)]
        return int(sum(sampvalues) / samples)

    def readToAngle(self, input_analog, raw=False):
        return (self.data_a + self.data_b * input_analog -
                (0 if raw else self.offset))

    def writeAngle(self, angle, raw=False, constrain=True):
        """
        Set the target angle of the servo

        Note that if the raw flag is set, the raw-angle is bound by the cooked-constraints

        :param angle: target angle in degrees
        :param raw: Do not apply the offset
        :param constrain: Keep the angle within the constraint (or within this range if specified)
        :return: None
        """
        if not raw:
            angle += self.offset
        angle = self.constrain(angle, constrain)
        self.connection.write(angle)

    def setConstraint(self, constraint):
        self.constraint = constraint

    def constrain(self, angle, constraint=True):
        if constraint is False:
            return angle
        if constraint is True:
            constraint = self.constraint
        # Angle is at most max(constraint) and at least min(constraint)
        angle = min(max(constraint), max(angle, min(constraint)))
        return angle

    def setServoParameters(self, offset=None, a=None, b=None):
        """
        Set the servo parameter
        :param offset: in range -25.5 - 25.5 degrees
        :param a: another offset in degrees
        :param b: a scale
        :return: None
        """
        if 1 <= self.servonumber <= 3:
            if offset is not None:
                self.setParameter(self.kAddrOffset + (self.servonumber - 1) * 2, offset*10, nbytes=2)
        if a is not None:
            self.setParameter(self.kAddrAandB + (self.servonumber - 1) * 6, a * 10, nbytes=3)
        if b is not None:
            self.setParameter(self.kAddrAandB + (self.servonumber - 1) * 6 + 3, b * 100, nbytes=3)

    def setParameter(self, addr, v, nbytes = 3):
        """
        Set parameter as bytes [sign, v%256, v//256] or [sign, v]
        :param addr: EEPROM address to write
        :param v:   value
        :param nbytes: 2 or 3
        :return: None
        """
        v = int(v)
        if v < 0:
            sig = 0
            v = -v
        else:
            sig = 1
        self.board.writeEEPROM(addr, sig)
        if nbytes == 3:
            self.board.writeEEPROM(addr+1, v % 256)
            self.board.writeEEPROM(addr+2, v//256)
        else:
            self.board.writeEEPROM(addr+1, v)

    def readServoOffset(self):
        if 1 <= self.servonumber <= 3:
            addr = self.kAddrOffset + (self.servonumber - 1) * 2
            servo_offset = (self.board.readEEPROM(addr + 1)) / 10.00
            if (self.board.readEEPROM(addr) == 0):
                servo_offset *= -1
        else:
            servo_offset = 0.0
        return servo_offset

    def readAandB(self):
        addr = self.kAddrAandB + (self.servonumber - 1) * 6

        data_a = (self.board.readEEPROM(addr + 1) + (self.board.readEEPROM(addr + 2) * 256)) / 10.0
        if (self.board.readEEPROM(addr) == 0):
            data_a = -data_a

        data_b = (self.board.readEEPROM(addr + 4) + (self.board.readEEPROM(addr + 5) * 256)) / 100.0
        if (self.board.readEEPROM(addr + 3) == 0):
            data_b = -data_b

        return data_a, data_b

    def __del__(self):
        # When this Servo object is garbage-collected, power down the physical servomotor
        self.detach()

class Uarm(object):
    # uarm = None

    def __init__(self, port, claw=False):
        self.uarm_status = 0
        # self.pin2_status = 0
        self.coord = {}
        self.g_interpol_val_arr = {}
        self.angle = {}
        self.claw = claw
        self.uarm = Arduino(port)
        self.assignServos()
        self.uarmDetach()

    def assignServos(self):
        self.servo_base = Servo(self.uarm, 1, 11, 2)
        self.servo_left = Servo(self.uarm, 2, 13, 0)
        self.servo_right = Servo(self.uarm, 3, 12, 1)
        self.servo_end = Servo(self.uarm, 4, 10, 3)
        self.servomap = {1: self.servo_base,
                         2: self.servo_left,
                         3: self.servo_right,
                         4: self.servo_end}
        if self.claw:
            self.servo_claw = Servo(self.uarm, 5, 9, 5)
            self.servomap[5] = self.servo_claw;

    def servos(self, servo_number=None):
        if servo_number is None:
            servo_number = list(self.servomap.keys())
        servo_number = self._aslist(servo_number)
        return [self.servomap[servo] for servo in servo_number]

    def servoAttach(self, servo_number=None, preserveAngle=True):
        for servo in self.servos(servo_number):
            servo.attach(preserveAngle=preserveAngle)

    def servoDetach(self, servo_number=None):
        for servo in self.servos(servo_number):
            servo.detach()
            self.uarm_status = 0

    def _aslist(self, v):
        try:
            return list(v)
        except TypeError:
            return [v]

    def uarmDisconnect(self, detach=True):
        if detach:
            self.servoDetach()
        self.uarm.exit()

    def uarmAttach(self):
        self.servoAttach()
        self.uarm_status = 1

    def uarmDetach(self):
        self.servoDetach()

    def angleConstrain(self, Angle):
        if Angle < 0:
            return 0
        elif Angle > 180:
            return 180
        else:
            return Angle

    def writeServoAngleRaw(self, servo_number, angle, constrain=False):
        self.servomap[servo_number].writeAngle(raw=True, constrain=constrain)

    def writeServoAngle(self, servo_number, angle, constrain=True):
        self.servomap[servo_number].writeAngle(raw=False, constrain=constrain)

    def writeAngle(self, *args, raw=False, constrain=True):
        for angle, servo in zip(args, self.servos()):
            servo.writeAngle(angle, raw=raw, constrain=constrain)

    def writeAngleRaw(self, *args, constrain=False):
        self.writeAngle(*args, raw=True, constrain=constrain)

    def readAnalog(self, servo_number=None):
        if servo_number is None or not hasattr(servo_number, 'len'):
            return [servo.readAnalog() for servo in self.servos(servo_number)]
        else:
            return self.servomap[servo_number].readAnalog()

    def readServoOffset(self, servo_number):
        if servo_number is None or not hasattr(servo_number, 'len'):
            return [servo.readOffset() for servo in self.servos(servo_number)]
        else:
            return self.servomap[servo_number].readServoOffset()

    def readToAngle(self, input_analog, servo_number, trigger):
        return self.servomap[servo_number].readToAngle(input_analog, raw=trigger)

    def fwdKine(self, theta_1, theta_2, theta_3):
        g_l3_1 = MATH_L3 * math.cos(theta_2 / MATH_TRANS)
        g_l4_1 = MATH_L4 * math.cos(theta_3 / MATH_TRANS);
        g_l5 = (MATH_L2 + MATH_L3 * math.cos(theta_2 / MATH_TRANS) + MATH_L4 * math.cos(theta_3 / MATH_TRANS));

        self.coord[1] = -math.cos(abs(theta_1 / MATH_TRANS)) * g_l5;
        self.coord[2] = -math.sin(abs(theta_1 / MATH_TRANS)) * g_l5;
        self.coord[3] = MATH_L1 + MATH_L3 * math.sin(abs(theta_2 / MATH_TRANS)) - MATH_L4 * math.sin(
            abs(theta_3 / MATH_TRANS));
        return self.coord

    def currentCoord(self):
        return self.fwdKine(self.readAngle(1), self.readAngle(2), self.readAngle(3))

    def currentX(self):
        self.currentCoord()
        return self.coord[1]

    def currentY(self):
        self.currentCoord()
        return self.coord[2]

    def currentZ(self):
        self.currentCoord()
        return self.coord[3]

    def readAngle(self, servo_number, raw=False):
        return self.servomap[servo_number].readAngle(raw=False)

    def readAngleRaw(self, servo_number):
        return self.readAngle(servo_number, raw=True)

    def interpolation(self, init_val, final_val):
        # by using the formula theta_t = l_a_0 + l_a_1 * t + l_a_2 * t^2 + l_a_3 * t^3
        # theta(0) = init_val; theta(t_f) = final_val
        # theta_dot(0) = 0; theta_dot(t_f) = 0

        l_time_total = 1

        l_a_0 = init_val;
        l_a_1 = 0;
        l_a_2 = (3 * (final_val - init_val)) / (l_time_total * l_time_total);
        l_a_3 = (-2 * (final_val - init_val)) / (l_time_total * l_time_total * l_time_total);

        i = 0
        while i < 51:
            l_t_step = (l_time_total / 50.0) * i
            self.g_interpol_val_arr[i] = l_a_0 + l_a_1 * (l_t_step) + l_a_2 * (l_t_step * l_t_step) + l_a_3 * (
                l_t_step * l_t_step * l_t_step);
            i += 1
        return self.g_interpol_val_arr

    def pumpOn(self):
        self.uarm.digital[PUMP_EN].write(1)
        self.uarm.digital[VALVE_EN].write(0)

    def pumpOff(self):
        self.uarm.digital[PUMP_EN].write(0)
        self.uarm.digital[VALVE_EN].write(1)
        time.sleep(0.02)
        self.uarm.digital[VALVE_EN].write(0)

    def ivsKine(self, x, y, z):
        if z > (MATH_L1 + MATH_L3 + TopOffset):
            z = MATH_L1 + MATH_L3 + TopOffset
        if z < (MATH_L1 - MATH_L4 + BottomOffset):
            z = MATH_L1 - MATH_L4 + BottomOffset

        g_y_in = (-y - MATH_L2) / MATH_L3
        g_z_in = (z - MATH_L1) / MATH_L3
        g_right_all = (1 - g_y_in * g_y_in - g_z_in * g_z_in - MATH_L43 * MATH_L43) / (2 * MATH_L43)
        g_sqrt_z_y = math.sqrt(g_z_in * g_z_in + g_y_in * g_y_in)

        if x == 0:
            # Calculate value of theta 1
            g_theta_1 = 90;
            # Calculate value of theta 3
            if g_z_in == 0:
                g_phi = 90
            else:
                g_phi = math.atan(-g_y_in / g_z_in) * MATH_TRANS
            if g_phi > 0:
                g_phi = g_phi - 180
            g_theta_3 = math.asin(g_right_all / g_sqrt_z_y) * MATH_TRANS - g_phi

            if g_theta_3 < 0:
                g_theta_3 = 0
            # Calculate value of theta 2
            g_theta_2 = math.asin((z + MATH_L4 * math.sin(g_theta_3 / MATH_TRANS) - MATH_L1) / MATH_L3) * MATH_TRANS
        else:
            # Calculate value of theta 1
            g_theta_1 = math.atan(y / x) * MATH_TRANS
            if (y / x) > 0:
                g_theta_1 = g_theta_1
            if (y / x) < 0:
                g_theta_1 = g_theta_1 + 180
            if y == 0:
                if x > 0:
                    g_theta_1 = 180
                else:
                    g_theta_1 = 0
            # Calculate value of theta 3
            g_x_in = (-x / math.cos(g_theta_1 / MATH_TRANS) - MATH_L2) / MATH_L3;
            if g_z_in == 0:
                g_phi = 90
            else:
                g_phi = math.atan(-g_x_in / g_z_in) * MATH_TRANS
            if g_phi > 0:
                g_phi = g_phi - 180

            g_sqrt_z_x = math.sqrt(g_z_in * g_z_in + g_x_in * g_x_in)

            g_right_all_2 = -1 * (g_z_in * g_z_in + g_x_in * g_x_in + MATH_L43 * MATH_L43 - 1) / (2 * MATH_L43)
            g_theta_3 = math.asin(g_right_all_2 / g_sqrt_z_x) * MATH_TRANS
            g_theta_3 = g_theta_3 - g_phi

            if g_theta_3 < 0:
                g_theta_3 = 0
            # Calculate value of theta 2
            g_theta_2 = math.asin(g_z_in + MATH_L43 * math.sin(abs(g_theta_3 / MATH_TRANS))) * MATH_TRANS

        g_theta_1 = abs(g_theta_1);
        g_theta_2 = abs(g_theta_2);

        if g_theta_3 < 0:
            pass
        else:
            self.fwdKine(g_theta_1, g_theta_2, g_theta_3)
            if (self.coord[2] > y + 0.1) or (self.coord[2] < y - 0.1):
                g_theta_2 = 180 - g_theta_2

        if (math.isnan(g_theta_1) or math.isinf(g_theta_1)):
            g_theta_1 = self.readAngle(1)
        if (math.isnan(g_theta_2) or math.isinf(g_theta_2)):
            g_theta_2 = self.readAngle(2)
        if (math.isnan(g_theta_3) or math.isinf(g_theta_3) or (g_theta_3 < 0)):
            g_theta_3 = self.readAngle(3)

        self.angle[1] = g_theta_1
        self.angle[2] = g_theta_2
        self.angle[3] = g_theta_3
        return self.angle

        pass

    def moveToWithS4(self, x, y, z, timeSpend, servo_4_relative, servo_4):

        if self.uarm_status == 0:
            self.uarmAttach()
            self.uarm_status = 1

        curXYZ = self.currentCoord()

        x_arr = {}
        y_arr = {}
        z_arr = {}

        if time > 0:
            self.interpolation(curXYZ[1], x)
            for n in range(0, 50):
                x_arr[n] = self.g_interpol_val_arr[n]

            self.interpolation(curXYZ[2], y)
            for n in range(0, 50):
                y_arr[n] = self.g_interpol_val_arr[n]

            self.interpolation(curXYZ[3], z)
            for n in range(0, 50):
                z_arr[n] = self.g_interpol_val_arr[n]

            for n in range(0, 50):
                self.ivsKine(x_arr[n], y_arr[n], z_arr[n])
                self.writeAngle(self.angle[1], self.angle[2], self.angle[3], self.angle[1] * servo_4_relative + servo_4)
                time.sleep(timeSpend / 50.0)

        elif time == 0:
            self.ivsKine(x, y, z)
            self.writeAngle(self.angle[1], self.angle[2], self.angle[3], self.angle[1] * servo_4_relative + servo_4)

        else:
            pass

    def moveTo(self, x, y, z):

        if self.uarm_status == 0:
            self.uarmAttach()
            self.uarm_status = 1

        curXYZ = self.currentCoord()

        x_arr = {}
        y_arr = {}
        z_arr = {}

        self.interpolation(curXYZ[1], x)
        for n in range(0, 50):
            x_arr[n] = self.g_interpol_val_arr[n]

        self.interpolation(curXYZ[2], y)
        for n in range(0, 50):
            y_arr[n] = self.g_interpol_val_arr[n]

        self.interpolation(curXYZ[3], z)
        for n in range(0, 50):
            z_arr[n] = self.g_interpol_val_arr[n]

        for n in range(0, 50):
            self.ivsKine(x_arr[n], y_arr[n], z_arr[n])
            self.writeAngle(self.angle[1], self.angle[2], self.angle[3], 0)

            time.sleep(0.04)

    def moveToWithTime(self, x, y, z, timeSpend):

        if self.uarm_status == 0:
            self.uarmAttach()
            self.uarm_status = 1

        curXYZ = self.currentCoord()
        x_arr = {}
        y_arr = {}
        z_arr = {}

        if time > 0:
            self.interpolation(curXYZ[1], x)
            for n in range(0, 50):
                x_arr[n] = self.g_interpol_val_arr[n]

            self.interpolation(curXYZ[2], y)
            for n in range(0, 50):
                y_arr[n] = self.g_interpol_val_arr[n]

            self.interpolation(curXYZ[3], z)
            for n in range(0, 50):
                z_arr[n] = self.g_interpol_val_arr[n]

            for n in range(0, 50):
                self.ivsKine(x_arr[n], y_arr[n], z_arr[n])
                self.writeAngle(self.angle[1], self.angle[2], self.angle[3], 0)
                time.sleep(timeSpend / 50.0)

        elif time == 0:

            self.ivsKine(x, y, z)
            self.writeAngle(self.angle[1], self.angle[2], self.angle[3], 0)

        else:
            pass

    def moveToAtOnce(self, x, y, z):

        if self.uarm_status == 0:
            self.uarmAttach()
            self.uarm_status = 1

        self.ivsKine(x, y, z)
        self.writeAngle(self.angle[1], self.angle[2], self.angle[3], 0)

    def moveRelative(self, x, y, z, time, servo_4_relative, servo_4):
        pass

    def stopperStatus(self):
        val = self.uarm.pumpStatus(0)

        if val == 2 or val == 1:
            return val - 1
        else:
            print
            'ERROR: Stopper is not deteceted'
            return -1
