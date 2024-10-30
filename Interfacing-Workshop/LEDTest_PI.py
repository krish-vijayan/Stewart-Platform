from smbus2 import SMBus

# Set bus address to 0x8
addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/i2c-1. Use this bus.

number = 1 # bool flag to stop while loop below

print("Type 1 for ON or 0 for OFF, then press the Enter key. To stop the program, type any other number, then press the Enter key")

# if user inputs an integer other than 1 or 0, program ends
while number == 1:
    ledstate = input(">>>>>>>     ")
    # Switch on
    if ledstate == "1":
	  # Sends a byte of data 0x1 to address ‘addr’ which is the Arduino Mega
        bus.write_byte(addr, 0x1)
    # Switch off
    elif ledstate == "0":
        bus.write_byte(addr, 0x0)
    # If input ledstate is not 0 or 1, while loop is broken
    else:
        number = 0
