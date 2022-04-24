import wiringpi
import time

INPUT = 0
OUTPUT = 1
LOW = 0
HIGH = 1
PUD_UP = 2

# initialize GPIO
wiringpi.wiringPiSetupGpio()

# index: motor number
# entry: GPIO number
relay_pin_LUT = [14, 15, 11, 9, 10, 8, 7, 4, 17, 18, 22, 23, 24, 25, 27, 0]

pin = relay_pin_LUT[0]

wiringpi.pinMode(pin, OUTPUT)
wiringpi.pullUpDnControl(pin, 0)
wiringpi.digitalWrite(pin, HIGH) # turn off relays

while(1):
	wiringpi.digitalWrite(pin, LOW)
	print('on')
	time.sleep(2)
	wiringpi.digitalWrite(pin, HIGH)
	print('off')
	time.sleep(2)

# # set pins to 1 (output)
# for pin in relay_pin_LUT:
# 	wiringpi.pinMode(pin, OUTPUT)
# 	wiringpi.pullUpDnControl(pin, PUD_UP)
# 	wiringpi.digitalWrite(pin, 1) # turn off relays

# while(1):
# 	for pin in relay_pin_LUT:
# 		wiringpi.digitalWrite(pin, LOW)
# 	print('on')
# 	time.sleep(2)
# 	for pin in relay_pin_LUT:
# 		wiringpi.digitalWrite(pin, HIGH)
# 	print('off')
# 	time.sleep(2)

# # driving the output to low turns the relay on
# while(1):
# 	# test one relay at a time
# 	for count, on_pin in enumerate(relay_pin_LUT):
# 		for off_pin in relay_pin_LUT:
# 			if off_pin != on_pin:
# 				wiringpi.digitalWrite(off_pin, 1)
# 		wiringpi.digitalWrite(on_pin, 0)
# 		print('Motor', count+1, 'on')
# 		time.sleep(2)