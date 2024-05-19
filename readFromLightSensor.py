import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")

def getAndUpdateColour():
	# Read the data from the sensor
        data = bus.read_i2c_block_data(0x44, 0x09, 6)
        # Insert code here

        # Convert the data to green, red and blue int values
        # Insert code here
        
        red = data[3] << 8 | data[2]
        green = data[1] << 8 | data[0]
        blue = data[5] << 8 | data[4]
        
        # red = data[3] + data[2]/256
        # green = data[1] + data[0]/256
        # blue = data[5] + data[4]/256
        
        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values
        print("RGB(%d %d %d)" % (red, green, blue))
        print()
        
        color = ""
        if green > blue & green > red:
            print("green")
        
        elif blue  > green & blue > red:
            print("blue")
        
        elif red > green & red > blue:
            print("red")
        
        time.sleep(2) 
for i in range(1000):
    getAndUpdateColour()
