import busio
import board
from adafruit_bus_device.i2c_device import I2CDevice
import time
import SX1509
import IO_Types

DEVICE_ADDRESS = 0x3E  # device address of SX1509

#Set up I2C
comm_port = busio.I2C(board.SCL, board.SDA)
device = I2CDevice(comm_port, DEVICE_ADDRESS)

#Initialize the expander
IOExpander = SX1509.SX1509(comm_port)
IOExpander.clock(oscDivider = 4)
IOExpander.debounceTime(32)


#Set up pins
for pin in range(16):
	IOExpander.pinMode(pin, IO_Types.PIN_TYPE_INPUT_PULLUP)



print('IO Expander Initialized')

while(1):
	print("------")
	for pin in range(16):
		print( IOExpander.digitalRead(pin) )
	time.sleep(.5)