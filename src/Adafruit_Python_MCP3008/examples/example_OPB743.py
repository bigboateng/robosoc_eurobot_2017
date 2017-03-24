# Simple example of reading the MCP3008 analog input channels and printing
# them all out.
# Author: Tony DiCola
# License: Public Domain
import time

# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008


# Software SPI configuration:
#CLK  = 18
#MISO = 23
#MOSI = 24
#CS   = 25
#mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))


print('Reading MCP3008 values, press Ctrl-C to quit...')
print('-' * 57)
count = 0
value = mcp.read_adc(7)
if (value > 900):
    state = 0
else :
    state = 1

# Main program loop.
while True:
    value = mcp.read_adc(7)
    #print(str(value))

    if (state==1 and value > 900):
    	state = 0
	count = (count+1)%120
	print("{}".format(count))
    elif (state==0 and value < 850):
	state = 1
	count = (count+1)%120
	print("{}".format(count))

    time.sleep(0.001)



