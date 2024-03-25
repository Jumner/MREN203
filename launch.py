import subprocess
import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Set GPIO pin for button
button_pin = 18
# Configure button pin with pull-up mode
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def button_callback(channel):
    print("Button pressed!")
    # Add event detection for button press
    try:
        subprocess.run("ros2 launch cbt_bringup cbt_launch.py", shell=True, check="true")
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nExecution interrupted.")
    except Exception as e:
        print(f"An error occurred: {e}")

GPIO.add_event_detect(button_pin, GPIO.RISING, callback=button_callback, bouncetime=200)
try:
    print("Press CTRL+C to exit")
    while True:
        time.sleep(0.1)  # Keep the script running
except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()  # Clean up GPIO on exit

