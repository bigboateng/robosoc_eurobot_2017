# encoding: utf-8
"""
SD21: Python class to access SD21 servo controller.
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

# Load Adafruit_I2C module if available (for BeagleBone and other platforms).
# On success set a flag.
try:
    from Adafruit_I2C import Adafruit_I2C
    adafruitAvailable = True
except ImportError:
    adafruitAvailable = False

def servoBaseReg(servo=1):
    """ Helper function to calculate the specified servo's base register. """
    if (servo<1) or (servo>21):
        servo = 1
    return 3*(servo-1)

def splitDoubleByte(data):
    """ Split a double byte value in single bytes list. """
    loByte = data & 0xFF
    hiByte = (data & 0xFF00) >> 8
    return (loByte, hiByte)

class SD21:
    """ Class for Devantech, Ltd. (a.k.a. robot-electronics) SD21 servo controller.
    
    Attributes
    ----------
    deviceAddress : int (byte)
        I2C address of SD21 device.
    PORT : str
        Serial port name (when using USBI2C device as I2C interface).
    SPEED : int
        Baudrate for serial port communication.
    driver : str
        Default connection driver ('dummy'). It prints packages to stdout.

    """
    

    def __init__(self):
        self.deviceAddress = 0xc2
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
            # TODO: Check succesful communication.
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
                  end=' | ')
            print(" ".join(str(ord(dataByte)) for dataByte in dataString))
            return chr(127)

    def connect(self, driver='USBI2C'):
        """ Connect to SD21 using specified driver. 

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
            # TODO: check Adafruit module docs for proper way to close connection.
            pass
        else: # dummy driver
            pass

    def setServoPulseWidth(self, servo=1, pulseWidth=1500):
        """ Set servo position by sending pulse width in us.
        
        The position is a 16 bit number which directly sets the output 
        pulse width in uS. Setting the position to 1500 (1500uS or 1.5mS) 
        will set most servo's to their center position. The range of pulse 
        widths that are normally supported are from 1000uS (1mS) to 2000uS 
        (2mS). It is usually possible to go beyond these limits though. 
        On a Hitec HS311 servo, we can set the position from 800 to 2200 to 
        give a nice wide range of movement. Take care though as its easy 
        to make the servo run into its internal stops if you give it pulse 
        widths at the upper or lower extremes. The registers can also be read 
        back (with getServoPulseWidth). 
        The position will be the current position of the servo during a 
        speed controlled movement, so you can track its progress towards 
        the requested position.

        Parameters
        ----------
        servo : int
            Servo number to control (1 .. 21).
        pulseWidth : int
            Pulse width in us, as an integer (16-bit max).

            
        """
        baseReg = servoBaseReg(servo)
        loReg   = baseReg + 1
        hiReg   = baseReg + 2
        loByte, hiByte = splitDoubleByte(pulseWidth)
        self.send(loReg, loByte)
        self.send(hiReg, hiByte)
        # TODO: return time to reach specified position if speed reg not 0.

    def setServoSpeed(self, servo=1, speed=0):
        """ Set servo speed. 

        The speed register controls the speed at which the servo moves to 
        its new position. The servo pulses are automatically refreshed 
        every 20mS. If the Speed register is zero (0x00) then the servo is 
        simply set to the requested position. On power up the Speed 
        registers are set to zero to give full speed, so unless you need 
        to slow them down the Speed registers can be ignored. If the Speed 
        register is set to something other than zero then that value is 
        added to the current position every 20mS until the target position 
        is reached. If you wish to move from 1000 to 2000 and the Speed 
        register is set to 10, then it will take 2 seconds to reach the set 
        position. The formula for the time it will take to make the move is:
        ((Target position-Start position)/Speed Reg)*20mS

        Parameters
        ----------
        servo : int
            Specified servo (1 .. 21).
        speed : int
            Speed value (defaults to 0).
        """
        baseReg = servoBaseReg(servo)
        self.send(baseReg, speed)

    def setServoPosition(self, servo=1, position=128):
        """ Set servo position by sending single byte. 

        To make things easier, the position can be set by writing a single 
        byte rather than two bytes for the pulse width in us. The processor 
        will multiply the number you write by 6 then add an offset of 732 
        and store the result as the pulse width. This gives you a range of 
        732 (0*6+732) to 2268 (256*6+732) in 6uS steps. The formula is:
        Base Reg*6+732uS

        The data is stored internally, and used with another two sets of 
        registers. These are positive and negative offsets. When you write 
        to the positive offset address the processor will add it to the 
        base position, multiply by 6 and add 732. It performs a similar 
        function for negative offsets. the formulas are:
        (BaseReg + PosReg) * 6 + 732 and
        (BaseReg - NegReg) * 6 + 732

        Parameters
        ----------
        servo : int
            Servo number (1 .. 21).
        position : int (byte)
            Requested position 0 to 255 (Defaults to 128).

        """
        baseReg  = 63
        servoReg = baseReg - 1 + servo
        self.send(baseReg, position)

    def setServoPosOffset(self, servo=1, position=0):
        """ Set positive position offset (see setServoPosition). 

        Parameters
        ----------
        servo : int
            Servo number (1 .. 21).
        position : int (byte)
            Positive offset: 0 to 255 (Defaults to 0).
        """
        baseReg  = 84
        servoReg = baseReg - 1 + servo
        self.send(baseReg, position)

    def setServoNegOffset(self, servo=1, position=0):
        """ Set negative position offset (see setServoPosition). 

        Parameters
        ----------
        servo : int
            Servo number (1 .. 21).
        position : int (byte)
            Negative offset: 0 to 255 (Defaults to 0).
        """
        baseReg  = 105
        servoReg = baseReg - 1 + servo
        self.send(baseReg, position)

    def getServoPulseWidth(self, servo=1):
        """ Get pulse width during speed controlled movement (see setServoPulseWidth). 

        Parameters
        ----------
        servo : int
            Servo number (1 ..21).

        Returns
        -------
        int
            Current pulse width in us.
        """
        baseReg = servoBaseReg(servo)
        loReg   = baseReg + 1
        hiReg   = baseReg + 2
        loByte  = self.recv(loReg)
        hiByte  = self.recv(hiReg)
        pulseWidth = (hiByte << 8) | loByte
        return pulseWidth

    def getBatteryLevel(self):
        """ Get battery level.

        the servo battery voltage in 39mV units up to a maximum of 10v. 
        A battery voltage of 7.2v will read about 184. 6v will read 
        about 154. It is updated every 20mS whether its read or not.
        """
        batteryReg = 65
        response = self.recv(batteryReg)

    def getVersion(self):
        """ Get software version. """
        versionReg = 45
        response = self.recv(versionReg)
