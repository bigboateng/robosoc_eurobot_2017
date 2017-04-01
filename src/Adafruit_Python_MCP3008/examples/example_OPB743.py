import time

# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

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

    if (state==1 and value > 900):
    	state = 0
	count = (count+1)%120
	print("{}".format(count))
    elif (state==0 and value < 850):
	state = 1
	count = (count+1)%120
	print("{}".format(count))

    time.sleep(0.001)



