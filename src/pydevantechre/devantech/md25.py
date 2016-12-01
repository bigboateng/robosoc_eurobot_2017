# encoding: utf-8
"""
MD25: Python class to access MD25 dual motor controller.
"""
# Copyright 2015 Daniel Vicente Lühr Sierra
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
from __future__ import print_function
from devantech.usbi2c import USBi2c
import time

# Load Adafruit_I2C module if available (for BeagleBone and other platforms).
# On success set a flag.
try:
    from Adafruit_I2C import Adafruit_I2C
    adafruitAvailable = True
except ImportError:
    adafruitAvailable = False

def accelTime(newSpeed, currentSpeed, accelRate):
    """ Helper function to calculate time to accelerate to new speed. """
    steps     = abs(newSpeed - currentSpeed) / accelRate
    totalTime = steps * 25.0 / 1000.0
    return totalTime

def signByte(unsigned):
    """ Helper function to transform unsigned byte to signed byte. """
    if unsigned > 0x7F:
        signed = (0xFF - unsigned) * (-1)
    else:
        signed = unsigned
    return signed

def sign4Bytes(unsigned):
    """ Helper function to transform unsigned 4-bytes number to signed int. """
    if unsigned > 0x7FFFFFFF:
        signed = (0xFFFFFFFF - unsigned) * (-1)
    else:
        signed = unsigned
    return signed

class MD25i2c:
    """ 
    Class for Devantech, Ltd. (a.k.a. robot-electronics) MD25 dual motor controller.

    Attributes
    ----------
    deviceAddress : int (byte)
        I2C address of MD25 device.
    PORT : str
        Serial port name (when using USBI2C device as I2C interface).
    SPEED : int
        Baudrate for serial port communication.
    driver : str
        Default connection driver ('dummy'). It prints packages to stdout.

    Note
    ----
    This class implements the i2c interface only.
    """
    

    def __init__(self):
        self.deviceAddress = 0xb0
        self.PORT   = '/dev/ttyUSB0'
        self.SPEED  = 19200
        self.driver = 'dummy'

    def send(self, register, data):
        """ Sends data byte to specified register. 

        Parameters
        ----------
        register : int
            Register to write data to.
        data : int (byte)
            Data byte to send.
        """
        if self.driver == 'USBI2C':
            self.usbModule.multiWrite(self.deviceAddress, 
                                      register, (data,) )
        elif (self.driver == 'Adafruit') and (adafruitAvailable):
            self.i2cModule.write8(register, data)
        else: # dummy driver
            dataString = ( chr(self.deviceAddress) + chr(register) 
                           + chr(1) + chr(data) )
            print('writing: ', end = '')
            print(" ".join(hex(ord(dataByte)) for dataByte in dataString),
                  end=' | ')
            print(" ".join(str(ord(dataByte)) for dataByte in dataString))


    def recv(self, register):
        """ Receive one byte from specified register.

        Parameters
        ----------
        register : int
            Register to read the byte from.

        Returns
        -------
        int (byte)
            Return read byte.
        """
        if self.driver == 'USBI2C':
            return self.usbModule.multiRead(self.deviceAddress, register, 1)
        elif (self.driver == 'Adafruit') and (adafruitAvailable):
            return self.i2cModule.readU8(register)
        else: #dummy driver
            dataString = (chr(self.deviceAddress) + chr(register) + chr(1))
            print('reading: ', end = '')
            print(" ".join(hex(ord(dataByte)) for dataByte in dataString),
                  end = ' | ')
            print(" ".join(str(ord(dataByte)) for dataByte in dataString))
            return chr(127)

    def connect(self, driver='USBI2C'):
        """ Connect to MD25 using specified driver. 

        Parameters
        ----------
        driver : str
            Driver to use: 'USBI2C' (default), 'Adafruit', 'dummy'

            'USBI2C' driver uses an USBI2C device and the usbi2c python 
            module to handle communication with the SD21.

            'Adafruit' driver uses Adafruit_I2C python module if 
            available (e.g. on the BeagleBone) to connect directly using 
            the I2C bus.

            'dummy' driver just print packages to stdout.

        Note
        ----
        Undefined drivers will default to the 'dummy' driver.
        """
        if driver == 'USBI2C':
            self.driver = driver
            self.usbModule = USBi2c()
            self.usbModule.connect(self.PORT)
        elif (driver == 'Adafruit') and (adafruitAvailable) :
            self.driver = driver
            self.i2cModule = Adafruit_I2C(self.deviceAddress)
        else:
            self.driver = 'dummy'

    def disconnect(self):
        """ Disconnect the device using the current driver. """
        if self.driver == 'USBI2C':
            self.usbModule.disconnect()
        elif (self.driver == 'Adafruit') and (adafruitAvailable):
            pass
        else: # dummy driver
            pass

    def setSingleMotorReg(self, motor=1, value=128):
        """ Set motor register to value. 

        Parameters
        ----------
        motor : int
            Motor number 1 or 2.
        value : int (byte)
            Motor register value to set.
        """
        if motor not in [1,2]:
            motor = 1
        else:
            # make sure it is an int
            motor = int(motor)
        motorReg  = motor - 1
        self.send(motorReg, value)
        
    def getSingleMotorReg(self, motor=1):
        """ Get motor register to value. 

        Parameters
        ----------
        motor : int
            Motor number 1 or 2.

        Returns
        -------
        int (byte)
            Motor register value.
        """
        if motor not in [1,2]:
            motor = 1
        else:
            # make sure it is an int
            motor = int(motor)
        motorReg = motor - 1
        response = self.recv(motorReg)
        return ord(response)

    def setDualMotorSpeed(self, speed1, speed2):
        """ Set the speed for both motors.

        Depending on the mode, the values set can be either 0 to 255 (mode 0) 
        or -128 to 127 (mode 1).

        Parameters
        ----------
        speed1 : int (byte)
            Speed value for motor 1.
        speed2 : int (byte)
            Speed value for motor 2.

        Returns
        -------
        list
            Returns times to reach desired speed for each motor (time1, time2).
        """
        mode            = self.getMode()
        actualMotorReg1 = self.getSingleMotorReg(1)
        actualMotorReg2 = self.getSingleMotorReg(2)
        accelRate       = self.getAccelRate()

        if mode == 0:
            if speed1 < 0:
                speed1 = 0
            elif speed1 > 255:
                speed1 = 255
            else:
                speed1 = int(speed1)
            if speed2 < 0:
                speed2 = 0
            elif speed2 > 255:
                speed2 = 255
            else:
                speed2 = int(speed2)
            actualSpeed1 = actualMotorReg1
            actualSpeed2 = actualMotorReg2
        elif mode == 1:
            if speed1 < -128:
                speed1 = -128
            elif speed1 > 127:
                speed1 = 127
            else:
                speed1 = int(speed1)
            if speed2 < -128:
                speed2 = -128
            elif speed2 > 127:
                speed2 = 127
            else:
                speed2 = int(speed2)
            actualSpeed1 = signByte(actualMotorReg1)
            actualSpeed2 = signByte(actualMotorReg2)
        else:
            # warn about wrong mode for command and discard
            return (-1, -1)
            pass

        self.setSingleMotorReg(1, speed1)
        self.setSingleMotorReg(2, speed2)

        time1 = accelTime(speed1, actualSpeed1, accelRate)
        time2 = accelTime(speed2, actualSpeed2, accelRate)

        return (time1, time2)

    def setMotorSpeedTurn(self, speed, turn):
        """ Set motors speed and turn values. 

        Turn mode looks at the speed register to decide if the direction 
        is forward or reverse. Then it applies a subtraction or addition 
        of the turn value on either motor.

        so if the direction is forward
        motor speed1 = speed - turn
        motor speed2 = speed + turn

        else the direction is reverse so
        motor speed1 = speed + turn
        motor speed2 = speed - turn

        If the either motor is not able to achieve the required speed 
        for the turn (beyond the maximum output), then the other motor 
        is automatically changed by the program to meet the required 
        difference.

        Parameters
        ----------
        speed : int (byte)
            Base speed value. 0 to 255 (mode 2), -128 to 127 (mode 3).
        turn : int (byte)
            Turn value. 0 to 255 (mode 2), -128 to 127 (mode 3).

        """
        mode = self.getMode()
        if mode == 2:
            if speed < 0:
                speed = 0
            elif speed > 255:
                speed = 255
            else:
                speed = int(speed)
            if turn < 0:
                turn = 0
            elif turn > 255:
                turn = 255
            else:
                turn = int(turn)
        if mode == 3:
            if speed < -128:
                speed = -128
            elif speed > 127:
                speed = 127
            else:
                speed = int(speed)
            if turn < 0:
                turn = 0
            elif turn > 255:
                turn = 255
            else:
                turn = int(turn)
        else:
            # warn about wrong mode for command and discard
            return

        self.setSingleMotorReg(1, speed)
        self.setSingleMotorReg(2, turn)


    def getEncoder(self, encoder=1):
        """ Get encoder value. 

        Parameters
        ----------
        encoder : int
            Encoder number 1 or 2.

        Returns
        -------
        int
            Encoder count value.
        """
        if encoder not in [1,2]:
            encoder = 1
        else:
            encoder = int(encoder)

        encoderWord = self.getEncoders()[encoder-1]

        return encoderWord

    def getEncoders(self):
        """ Read both encoders.

        Returns
        -------
        list
            List of both encoders' count values and raw encoder data (8 bytes).
            (encoder1, encoder2, rawdata)

        """
        encoderBaseReg = 0x02
        encodersBytes = self.usbModule.multiRead(self.deviceAddress, encoderBaseReg, 8)
        encoder1 = ((ord(encodersBytes[0]) << 3*8) + 
                    (ord(encodersBytes[1]) << 2*8) +
                    (ord(encodersBytes[2]) << 1*8) +
                    (ord(encodersBytes[3])))
        encoder2 = ((ord(encodersBytes[4]) << 3*8) + 
                    (ord(encodersBytes[5]) << 2*8) +
                    (ord(encodersBytes[6]) << 1*8) +
                    (ord(encodersBytes[7])))
        return (sign4Bytes(encoder1), sign4Bytes(encoder2), encodersBytes)

    def getBatteryVoltage(self):
        """ Get the supply battery voltage. 

        Returns
        -------
        float
            Reading of the voltage of the connected battery in volts.
        """
        batteryReg = 10
        response = self.recv(batteryReg)
        return ord(response) / 10.0

    def getMotorCurrent(self, motor=1):
        """ The current through motor. 

        Parameters
        ----------
        motor : int
            motor number 1 or 2.

        Returns
        -------
        float
            Approximate current in amps.
        """
        if motor not in [1,2]:
            motor = 1
        else:
            motor = int(motor)
        
        baseReg  = 11
        motorReg = motor - 1 + baseReg
        response = self.recv(motorReg)
        return ord(response) / 10.0

    def getMotorCurrents(self):
        """ Read both motors currents. 

        Returns
        -------
        List
            Both motor currents in amps (motorCurrent1, motorCurrent2).
        """
        motorCurrent1 = self.getMotorCurrent(1)
        motorCurrent2 = self.getMotorCurrent(2)
        return (motorCurrent1, motorCurrent2)

    def getSoftwareRevision(self):
        """ Get Software Revision Number. 

        Returns
        -------
        int
            Software Revision Number.
        """
        softwareReg = 13
        response = self.recv(softwareReg)
        return ord(response)

    def setAccelRate(self, accel=5):
        """ Set acceleration rate.

        If you require a controlled acceleration period for the attached 
        motors to reach there ultimate speed, the MD25 has a register to 
        provide this. It works by using a value into the acceleration 
        register and incrementing the power by that value. Changing 
        between the current speed of the motors and the new speed. So if 
        the motors were traveling at full speed in the forward direction 
        (255) and were instructed to move at full speed in reverse (0), 
        there would be 255 steps with an acceleration register value of 1, 
        but 128 for a value of 2. The default acceleration value is 5, 
        meaning the speed is changed from full forward to full reverse 
        in 1.25 seconds. The register will accept values of 1 up to 10 
        which equates to a period of only 0.65 seconds to travel from full 
        speed in one direction to full speed in the opposite direction.

        So to calculate the time (in seconds) for the acceleration to 
        complete :

        if new speed > current speed
        steps = (new speed - current speed) / acceleration register

        if new speed < current speed
        steps = (current speed - new speed) / acceleration register

        time = steps * 25ms 

        Parameters
        ----------
        accel : int
            Acceleration rate value.
        """
        if (accel<1) or (accel>10):
            accel = 5
        else:
            accel = int(accel)

        accelReg = 14

        self.send(accelReg, accel)

    def getAccelRate(self):
        """ Get current acceleration rate. 

        Returns
        -------
        int
            Current acceleration rate.
        """
        accelReg = 14
        response = self.recv(accelReg)
        return ord(response)

    def setMode(self, mode=0):
        """ Set mode of operation.

        The mode register selects which mode of operation and I2C data 
        input type the user requires. The options being:
        0,    (Default Setting) If a value of 0 is written to the mode 
        register then the meaning of the speed registers is literal speeds 
        in the range of 0 (Full Reverse)  128 (Stop)   255 (Full Forward).

        1,    Mode 1 is similar to Mode 0, except that the speed registers 
        are interpreted as signed values. The meaning of the speed registers 
        is literal speeds in the range of:
        -128 (Full Reverse)   0 (Stop)   127 (Full Forward).

        2,    Writing a value of  2 to the mode register will enable 
        speed + turn control of both motors. Data is in the range of:
        0 (Full Reverse)  128 (Stop)  255 (Full  Forward).

        3,    Mode 3 is similar to Mode 2, except that the speed registers 
        are interpreted as signed values. Data is in the range of:
        -128  (Full Reverse)  0 (Stop)   127 (Full Forward) 

        """
        if mode not in [0, 1, 2, 3]:
            mode = 0
        else:
            mode = int(mode)

        modeReg = 15

        self.send(modeReg, mode)

    def getMode(self):
        """ Get current mode. 

        Returns
        -------
        int
            Current mode of operation.
        """
        modeReg  = 15
        response = self.recv(modeReg)
        return ord(response)

    def resetEncoders(self):
        """ Reset encoders' count to zero. """
        commandReg = 16
        command    = 32
        self.send(commandReg, command)

    def disableSpeedRegulation(self):
        """ Disables automatic speed regulation.

        By using feedback from the encoders the MD25 is able to 
        dynamically increase power as required. If the required 
        speed is not being achieved, the MD25 will increase power 
        to the motors until it reaches the desired rate or the 
        motors reach there maximum output. Speed regulation can 
        be turned off.
        """
        commandReg = 16
        command    = 48
        self.send(commandReg, command)


    def enableSpeedRegulation(self):
        """ Enables automatic speed regulation (see disableSpeedRegulation). """
        commandReg = 16
        command    = 49
        self.send(commandReg, command)

    def disableMotorTimeout(self):
        """ Disable automatic motor timeout. 

        The MD25 will automatically stop the motors if there is no 
        I2C communications within 2 seconds. This is to prevent your 
        robot running wild if the controller fails. The feature can 
        be turned off, if not required.
        """
        commandReg = 16
        command    = 50
        self.send(commandReg, command)

    def enableMotorTimeout(self):
        """ Enable automatic motor timeout (see disableMotorTimeout). """
        commandReg = 16
        command    = 51
        self.send(commandReg, command)

    def changeI2Caddress(self, newAddress=0xB0):
        """ Change I2C address. 

        To change the I2C address of the MD25 by writing a new address 
        you must have only one module on the bus. When done, you should 
        label the MD25 with its address, however if you do forget, just 
        power it up without sending any commands. The MD25 will flash its 
        address out on the green communication LED. One long flash 
        followed by a number of shorter flashes indicating its address. 
        Any command sent to the MD25 during this period will still be 
        received and writing new speeds or a write to the command 
        register will terminate the flashing.
        1 long flash and 0 short flashes: Address 176(dec) 0xB0(hex)
        1 long flash and 1 short flash:   Address 178(dec) 0xB2(hex)
        1 long flash and 2 short flashes: Address 180(dec) 0xB4(hex)
        1 long flash and 3 short flashes: Address 182(dec) 0xB6(hex)
        1 long flash and 4 short flashes: Address 184(dec) 0xB8(hex)
        1 long flash and 5 short flashes: Address 186(dec) 0xBA(hex)
        1 long flash and 6 short flashes: Address 188(dec) 0xBC(hex)
        1 long flash and 7 short flashes: Address 190(dec) 0xBE(hex)

        """
        addressList = range(0xb0, 0xbe+1, 2)
        if newAddress not in addressList:
            newAddress = addressList[0]

        commandReg = 16
        self.send(commandReg, 160)
        time.sleep(0.005)
        self.send(commandReg, 170)
        time.sleep(0.005)
        self.send(commandReg, 165)
        time.sleep(0.005)
        self.send(commandReg, newAddress)
