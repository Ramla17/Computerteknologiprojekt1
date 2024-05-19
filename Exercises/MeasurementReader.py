# coding: utf-8
import time
import RPi.GPIO as GPIO

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO to use on Pi
GPIO_TRIGECHO = 15  # Used for the LV-PROXSONAR-EZ SENSOR
GPIO_LED = 17   # Used for the LED

print("Ultrasonic Measurement")

# Set pins as output and input
GPIO.setup(GPIO_TRIGECHO,GPIO.OUT)  # Initial state as output
GPIO.setup(GPIO_LED, GPIO.OUT)       # LED pin as output


# Set trigger to False (Low)
GPIO.output(GPIO_TRIGECHO, False)

def measure():
  # This function measures a distance
  # Pulse the trigger/echo line to initiate a measurement
    GPIO.output(GPIO_TRIGECHO, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGECHO, False)
  #ensure start time is set in case of very quick return
    start = time.time()

  # set line to input to check for start of echo response
    GPIO.setup(GPIO_TRIGECHO, GPIO.IN)
    while GPIO.input(GPIO_TRIGECHO)==0:
        start = time.time()

  # Wait for end of echo response
    while GPIO.input(GPIO_TRIGECHO)==1:
        stop = time.time()
  
    GPIO.setup(GPIO_TRIGECHO, GPIO.OUT)
    GPIO.output(GPIO_TRIGECHO, False)

  # This part calculates the distance
    elapsed = stop-start
    distance = (elapsed * 34300)/2.0
    time.sleep(0.1)
    return distance



try:
    while True:
        distance = measure()
        print("Distance: %.1f cm" % distance)
        
        # LED control based on distance
        # The print-functions makes it easier to read and understand outputs on the terminalg
        if distance < 18:
          print("Distancen er under 18cm") 
          for i in range(0,5): #we made this for loop for it to blink constantly when under 18 cm
                GPIO.output(GPIO_LED, GPIO.HIGH)
                time.sleep(0.2) #wait 0.2 second before next measurement
                GPIO.output(GPIO_LED, GPIO.LOW)
                time.sleep(0.2)
                
                
        elif 18 <= distance < 25:
            print("Distancen er mellem 18cm og 25cm")
            GPIO.output(GPIO_LED, GPIO.LOW)
            time.sleep(1)
            GPIO.output(GPIO_LED, GPIO.HIGH)
            time.sleep(1)
            
        elif 25 <= distance <= 30:
            print("Distancen er mellem 25cm og 30cm")
            GPIO.output(GPIO_LED, GPIO.LOW)
            time.sleep(2)
            GPIO.output(GPIO_LED, GPIO.HIGH)
            time.sleep(2)
            
        else:
            print("Distancen er større end 30cm")
            GPIO.output(GPIO_LED, GPIO.LOW)
            
        time.sleep(1)

except KeyboardInterrupt:
    print("Stop")
    GPIO.cleanup()

