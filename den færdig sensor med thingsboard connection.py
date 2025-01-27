from hcsr04 import HCSR04
from time import sleep
from machine import Pin, PWM
from client2 import TBDeviceMqttClient
import secrets
import connect 


wlan = connect.do_connect()


client = TBDeviceMqttClient(secrets.SERVER_IP_ADDRESS, access_token=secrets.ACCESS_TOKEN)

try:
    client.connect()
    print("Connected to ThingsBoard")
except Exception as e:
    print(f"Failed to connect to ThingsBoard: {e}")


usonic = HCSR04(trigger_pin=15, echo_pin=35, echo_timeout_us=10000)


led1 = Pin(19, Pin.OUT)
led2 = Pin(4, Pin.OUT) 


buzzer = PWM(Pin(14, Pin.OUT), duty=0) 

def blink_leds(delay_time):
    
    led1.on()
    led2.on()
    sleep(0.5)  
    led1.off()
    led2.off()
    sleep(delay_time - 0.5) 


while True:
    try:
        
        distance = int(usonic.distance_cm())
        print('Distance:', distance, 'cm')

        
        telemetry = {"distance": distance}
        client.send_telemetry(telemetry)

        
        if 0 <= distance <= 50:
            blink_leds(1) 
            buzzer.duty(600)
            buzzer.freq(700) 
            print("PAS PÅ ")
            
            alarm_data = {"alert": "Danger!!"}
            client.send_telemetry(alarm_data)
        elif 51 <= distance <= 100:
            blink_leds(2)
            buzzer.duty(0)
        elif 101 <= distance <= 150:
            blink_leds(3)
            buzzer.duty(0)
        else:
            
            led1.off()
            led2.off()
            buzzer.duty(0)
            sleep(1) 

    except OSError as e:
        print("Sensorfejl:", e)
        led1.off()
        led2.off()
        buzzer.duty(0)  
        sleep(1)

