# encoding: utf-8
"""
USBI2C: Python class to access USBI2C interface module.
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

import serial

def bool2int(bitList):
    """ Transform boolean list to integer value. """
# taken from http://stackoverflow.com/questions/4065737/python-numpy-convert-list-of-bools-to-unsigned-int/4065901#4065901
    byteValue = 0
    for i, b in enumerate(x):
        if b: y += (1 << i)
    return byteValue

def byte2bool(byteValue):
    """ Transform byte value to list of bools. """
    bitList = list()
    for i in range(8):
        bitlist.append(bool(byteValue & 2**i >> i))
    return bitlist

class USBi2c:
    """ Class for Devantech, Ltd. (a.k.a. robot-electronics) USB-to-I2C module"""

    def connect(self, port='/dev/ttyUSB0'):
        """ Connect to specified serial port with default serial settings.

        This method uses pySerial to connect to the USB port where the module 
        is connected. It uses 19200 baud, and 8N2 serial settings. Timeout is 
        set to 1s although Devantech's docs recomend, at least, 500ms.

        It DOES NOT autodetect the port.

        Parameters
        ----------
        port : str, optional
            The serial port to connect (defaults to '/dev/ttyUSB0').
 
        """
        self.module = serial.Serial(port, baudrate=19200, 
                                    stopbits=serial.STOPBITS_TWO, timeout=1)

    def disconnect(self):
        """ Close serial connection. """
        self.module.close()

    def singleWrite(self, deviceAddr, dataByte):
        """ Write single byte for non-registered devices.

        Parameters
        ----------
        deviceAddr : int
            Target device's I2C address. Should be an 8-bit integer (byte).
        dataByte : int
            Data byte to write to the target device.
            

        Returns
        -------
        bool
            True if succesful, False otherwise.
        """

        # Assemble package: | command (0x53) | i2c address | data |
        payload='\x53'+chr(deviceAddr)+chr(dataByte)
        # Send the package.
        self.module.write(payload)
        # Retrieve the result: 0 = Failure.
        exitStatus = self.module.read()
        if exitStatus == 0x00:
            return False
        else:
            return True

    def singleRead(self, deviceAddr):
        """ Read single byte for non-registered devices.

        Parameters
        ----------
        deviceAddr : int
            Target device's I2C address.
        Returns
        -------
        str
            Read byte as returned by pySerial read() method.
        """
        # Assemble package: | command (0x53) | i2c address + read bit |
        payload='\x53'+chr(deviceAddr+1)
        self.module.write(payload)
        # Receive one byte and return it.
        response = self.module.read(1)
        return response

    def noNewAddressMultiRead(self, deviceAddr, dataSize):
        """ Read multiple bytes without setting new address.

        This is used for devices that do not have an internal register 
        address but returns multiple bytes. Examples of such devices 
        are the Honeywell ASDX DO series pressure sensors. This command
        can also be used for devices that do have an internal address 
        which it increments automatically between reads and doesn't need 
        to be set each time, such as eeproms. In this case you would use 
        command multiRead or multiRead2 for the first read, then I2C_MUL 
        for subsequent reads.

        Parameters
        ----------
        deviceAddr : int
            Target device's I2C address. Should be an 8-bit integer (byte).
        deviceReg : int
            Target device's register address. 8-bit integer
        dataSize : int
            Number of bytes to read.
            
        Returns
        -------
        str
            Returned values as string (char sequence).

        """
        payload = '\x54' + chr(deviceAddr+1)
        self.module.write(payload)
        response = self.module.read(dataSize)
        return response


    def multiWrite(self, deviceAddr, deviceReg, dataBytes):
        """ Write single or multiple bytes for 1 byte addressed devices.

        The maximum number of data bytes should not exceed 64 so as not 
        to overflow the USB-I2C's internal buffer.

        Parameters
        ----------
        deviceAddr : int
            Target device's I2C address. Should be an 8-bit integer (byte).
        deviceReg : int
            Target device's register address. 8-bit integer
        dataBytes : list
            List of data bytes to write to the target device. 
            
        Returns
        -------
        bool
            True if succesful, False otherwise.

        """
        dataSize = len(dataBytes)
        dataString = "".join(chr(dataByte) for dataByte in dataBytes)
        payload = ('\x55' + chr(deviceAddr) + chr(deviceReg) + chr(dataSize) 
                   + dataString)
        self.module.write(payload)
        exitStatus = self.module.read(1)
        if exitStatus == 0x00:
            return False
        else:
            return True


    def multiRead(self, deviceAddr, deviceReg, dataSize=1):
        """ Write single or multiple bytes for 1 byte addressed devices.

        The maximum number of data bytes requested should not exceed 60 
        so as not to overflow the USB-I2C's internal buffer.

        Parameters
        ----------
        deviceAddr : int
            Target device's I2C address. Should be an 8-bit integer (byte).
        deviceReg : int
            Target device's register address. 8-bit integer
        dataSize : int
            Number of bytes to read.
            
        Returns
        -------
        str
            Read values as a sequence of chars

        """

        payload = '\x55' + chr(deviceAddr+1) + chr(deviceReg) + chr(dataSize)
        self.module.write(payload)
        response = self.module.read(dataSize)
        return response

    def multiWrite2(self, deviceAddr, deviceReg, dataBytes):
        """ Write single or multiple bytes for 2 byte addressed devices.

        Parameters
        ----------
        deviceAddres : int
            Target device's I2C address. Should be an 8-bit integer (byte).
        deviceReg : int
            Target device's register address. 16-bit integer
        dataBytes : list
            List of data bytes to write to the target device. 
            
        Returns
        -------
        bool
            True if succesful, False otherwise.

        """
        dataSize = len(dataBytes)
        dataString = "".join(chr(dataByte) for dataByte in dataBytes)
        deviceRegHi = (0xFF00 & deviceReg) >> 8
        deviceRegLo = 0x00FF & deviceReg
        payload = ('\x56' + chr(deviceAddr) + chr(deviceRegHi)
                   + chr(deviceRegLo) + chr(dataSize) + dataString)
        self.module.write(payload)
        exitStatus = self.module.read(1)
        if exitStatus == 0x00:
            return False
        else:
            return True

    def multiRead2(self, deviceAddr, deviceReg, dataSize):
        """ Write single or multiple bytes for 2 byte addressed devices.

        The maximum number of data bytes requested should not exceed 64 
        so as not to overflow the USB-I2C's internal buffer.

        Parameters
        ----------
        deviceAddr : int
            Target device's I2C address. Should be an 8-bit integer (byte).
        deviceReg : int
            Target device's register address. 16-bit integer
        dataSize : int
            Number of bytes to read.
            
        Returns
        -------
        str
            Read values as a sequence of chars

        """
        deviceRegHi = (0xFF00 & deviceReg) >> 8
        deviceRegLo = 0x00FF & deviceReg
        payload = ('\x56' + chr(deviceAddr+1) + chr(deviceRegHi) 
                   + chr(deviceRegLo) + chr(dataSize))
        self.module.write(payload)
        response = self.module.read(dataSize)
        return response

    def usb(self, USBcommand, data01, data02):
        """ Send command to USB-I2C module.

        A range of commands to the USB-I2C module, generally to 
        improve selected communications or provide analogue/digital I/O.

        Parameters
        ----------
        USBcommand : int
            Command to send to the module.
        data01 : int
            First data byte.
        data02 : int
            Second data byte

        Returns
        -------
        int or list of ints
            Return one or more bytes depending on the command.

        """
        # drop invalid values
        if (USBcommand < 1) or (USBcommand > int(0x12)):
            USBcommand = 1
        expectedBytes=(1,1,1,6,9,12,15,21,27,39,51,1,1,4)
        payload = '\x5a'+chr(USBcommand)+chr(data01)+chr(data02)
        self.module.write(payload)
        response = self.module.read(expectedBytes[USBcommand])
        return response

    def getRevision(self):
        """ Returns the USB-I2C firmware revision number.

        Returns
        -------
        int
            USB-I2C firmware revision number.

        """
        command = 0x01
        data01  = 0x00
        data02  = 0x00
        revision = self.usb(command, data01, data02)
        return revision

    def setPins(self, io3=True, io2=True, redLed=True):
        """ Sets I/O pins high/low.

        Notes
        -----
        I/O mode and I2C mode cannot be mixed, I/O commands should 
        not be used when I2C devices are connected. 
        
        If the USB-I2C module is not being used for I2C, it can be 
        used as general purpose I/O controller with three I/O lines. 
        Input 1 is always an input only pin and has a 47k pull-up 
        resistor (not 4.7k like the others). The other two can be input 
        or output. The outputs are set high/low with the SETPINS command. 
        The pin is not actively driven high, it is released and pulled 
        high by a 4.7k resistor. Output low is actively driven and can 
        sink a maximum of 24mA. GETPINS will return the status of the 
        I/O pins. To use an I/O pin as an input, it must first have 
        a 1 (high) written to it. This will release the pin so that 
        the 4.7k resistor will pull it high, it can then be used as 
        an input. Both SETPINS and GETPINS commands will return the 
        status of the I/O Pins, however, only SETPINS can change them.

        Parameters
        ----------
        io3 : bool
            I/O_3 status value.
        io2 : bool
            I/O_2 status value.
        redLed : bool
            Red Led ON/OFF.

        Returns
        -------
        List of boolean values
            Returns the status of the pins (Red_Led, I_1, I/O_2, I/O_3)

        """
        command = 0x10
        data01  = bool2int(redLed, True, io2, io3)
        data02  = 0x00
        pinsByte = self.usb(command, data01, data02)
        return byte2bool(0x0F & pinsByte)

    def getPins(self):
        """ Gets the status of I/O pins.

        Notes
        -----
        
        Read notes for the setPins command.

        Returns
        -------
        List of boolean values
            Returns the status of the pins (Red_Led, I_1, I/O_2, I/O_3)
        
        """
        command = 0x11
        data01  = 0x00
        data02  = 0x00
        pinsByte = self.usb(command, data01, data02)
        return byte2bool(0x0F & pinsByte)

    def getAD(self):
        """ Gets Analogue value on I/O2 and I/O3.

        The USB-I2C module can also convert the analogue values on 
        pins I/O2 and I/O3. Before doing this the I/O pins should be 
        set high, effectively making them inputs. Remember though that 
        this is primarily a USB to I2C interface and as such has 4k7 
        pull-up resistors. Take this into account when connecting your 
        analogue input.
        
        Returns
        -------
        List of ints
            Returns AD values (I/O_2 AD, I/O_3 AD)

        """
        command = 0x12
        data01  = 0x00
        data02  = 0x00
        adBytes = self.usb(command, data01, data02)
        io2Hi   = adBytes(0)
        io2Lo   = adBytes(1)
        io3Hi   = adBytes(2)
        io3Lo   = adBytes(3)
        io2     = (io2Hi << 8) | io2Lo
        io3     = (io3Hi << 8) | io3Lo
        return (io2, io3)

    def newAddress(self, newAddr):
        """ Changes SRF08 I2C address.

        This command is used to change an SRF08's I2C address to a 
        different address. Changing the address on the SRF08 
        requires 4 separate transactions on the I2C bus. The USB-I2C 
        know how to change an SRF08's I2C address and just needs you 
        to send it the new address using this command. When using it, 
        make sure you only have one SRF08 connected, otherwise you will 
        set every SRF08 on the bus to the same address. The single 
        return byte is the new address sent back when the task is complete.

        Parameters
        ----------
        newAddr : int
            New SRF08's I2C address.

        Returns
        -------
        int
            New SRF08's I2C address.

        """
        command = 0x02
        data01  = newAddr
        data02  = 0x00
        newSetAddress = self.usb(command, data01, data02)
        return newSetAddress

    def scan(self, sonars, speedLeft, speedRight):
        """ Send motor data - return battery, compass & sonar data.

        This command is provided for CM02 compatibility. It assumes 
        you have an MD22 motor controller, a CMPS03 compass module 
        and a number of SRF08 rangefinders (1, 2, 3, 4, 6, 8, 12 or 16).
        After sending the new motor speeds to the MD22, the USB-I2C 
        will send a return frame comprising the battery voltage 
        (reads 0x00). This is followed by two bytes of compass 
        bearing - high byte first, and then three bytes for each SRF08. 
        The first of the three bytes is the SRF08's light sensor reading. 
        The next two bytes is the range - high byte first. 

        SRF08 data is always returned starting with address 0xE0, 0xE2, 
        0xE4 - going up one address at a time until all requested SRF08's 
        data has been sent.

        After sending the data back up to the PC, the USB-I2C automatically 
        issues a new ranging command to all SRF08s. The ranging 
        command used is 82 (0x52) which returns the results in uS. 
        To convert to cm divide by 58 and to convert to inches 
        divide by 148.

        SRF08 addresses should have been set up before running this 
        command and the MD22 should be initialized to the mode and 
        acceleration required. One more important feature. The SCAN 
        command also sets up a 500mS timer on the USB-I2C. If another 
        SCAN command is not received within this time, a command is 
        automatically sent to the MD22 to stop the motors. This is to 
        prevent your robot wandering out of control if it ventures 
        outside of the range of the radio link. 

        Parameters
        ----------
        sonars : int
            Number of SRF08 sonars connected. Must be in [1, 2, 3, 4, 6, 8, 12, 16].
        speedLeft : int
            Byte indicating MD22 speed for left motor.
        speedRight : int
            Byte indicating MD22 speed for right motor.

        Returns
        -------
        List of ints
            Returns frame according to description.

        """
        expectedSonars = [1, 2, 3, 4, 6, 8, 12, 16]
        if sonars not in expectedSonars:
            sonars = 1
        command = 0x04 + expectedSonars.index(sonars)
        data01  = speedLeft
        data02  = speedRight
        returnFrame = self.usb(command, data01, data02)
        return returnFrame
