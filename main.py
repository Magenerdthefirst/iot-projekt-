from machine import Pin, UART, I2C, PWM
from time import sleep
import dht
from gps_simple import GPS_SIMPLE
from gpio_lcd import GpioLcd
from ina219_lib import INA219
from hcsr04 import HCSR04
from client2 import TBDeviceMqttClient
import secrets
import connect
from mpu6050 import MPU6050
from neopixel import NeoPixel
###################################### her har vi konfiguret alle vores sensor i vores main.py såsom lcd,gps,dht11,ina219,mpu6080 og hc-sr04, buzzer og led 
n = 16
p = 12
np = NeoPixel(Pin(p, Pin.OUT), n)

client = TBDeviceMqttClient(secrets.SERVER_IP_ADDRESS, access_token=secrets.ACCESS_TOKEN)

try:
    client.connect()
    print("Connected to ThingsBoard")
except Exception as e:
    print(f"Failed to connect to ThingsBoard: {e}")

dht11_pin = 0
dht11 = dht.DHT11(Pin(dht11_pin))

gps_port = 2
gps_speed = 9600
uart = UART(gps_port, gps_speed)
gps = GPS_SIMPLE(uart)

i2c = I2C(0, scl=Pin(22), sda=Pin(21))
ina = INA219(i2c)

lcd = GpioLcd(
    rs_pin=Pin(27), enable_pin=Pin(25),
    d4_pin=Pin(33), d5_pin=Pin(32),
    d6_pin=Pin(21), d7_pin=Pin(22),
    num_lines=4, num_columns=20, backlight_pin=Pin(23, Pin.OUT)
)
lcd.clear()

i2c_imu = I2C(0)
imu = MPU6050(i2c_imu)

usonic = HCSR04(trigger_pin=15, echo_pin=35, echo_timeout_us=10000)

led1 = Pin(19, Pin.OUT)
led2 = Pin(4, Pin.OUT)
buzzer = PWM(Pin(14, Pin.OUT), duty=0)

BATTERY_CAPACITY_MAH = 1800
VOLTAGE_MIN = 3.0
VOLTAGE_MAX = 4.2
############################################################## 
def calculate_capacity(voltage, min_voltage, max_voltage):
    if voltage < min_voltage:
        return 0
    elif voltage > max_voltage:
        return 100
    else:
        return ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def send_telemetry(data):
    try:
        client.send_telemetry(data)
        print(f"Telemetry sent: {data}")
    except Exception as e:
        print(f"Failed to send telemetry: {e}")

def blink_leds(delay_time):
    led1.on()
    led2.on()
    sleep(0.5)
    led1.off()
    led2.off()
    sleep(delay_time - 0.5)

while True:
    latitude, longitude, speed, course = "N/A", "N/A", "N/A", "N/A"
    temp, humidity, current, battery_percentage = "N/A", "N/A", "N/A", 0
    acceleration_x = "N/A"
    distance = "N/A"

    if gps.receive_nmea_data():
        latitude = gps.get_latitude()
        longitude = gps.get_longitude()
        try:
            speed_knots = gps.get_speed()
            speed = round(speed_knots * 1.852, 2)
        except TypeError:
            speed = "N/A"
        course = gps.get_course()

    try:
        dht11.measure()
        temp = dht11.temperature()
        humidity = dht11.humidity()
    except OSError as e:
        print("Error reading DHT11:", e)

    try:
        voltage = ina.get_bus_voltage()
        shunt_voltage = ina.get_shunt_voltage()
        current = shunt_voltage / 0.1
        battery_percentage = calculate_capacity(voltage, VOLTAGE_MIN, VOLTAGE_MAX)
    except Exception as e:
        print("INA219 error:", e)

    try:
        values = imu.get_values()
        acceleration_x = values['acceleration x']
        acceleration_x = max(0, min(acceleration_x, 15000))
        red_intensity = map_value(acceleration_x, 0, 15000, 0, 255)

        for i in range(n):
            np[i] = (red_intensity, 0, 0)
        np.write()
    except Exception as e:
        print("MPU6050 error:", e)

    try:
        distance = int(usonic.distance_cm())
        print(f"Distance: {distance} cm")

        if 0 <= distance <= 50:
            blink_leds(1)
            buzzer.duty(600)
            buzzer.freq(700)
            print("Danger: Close")
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

    telemetry_data = {
        "latitude": latitude,
        "longitude": longitude,
        "speed": speed,
        "course": course,
        "temperature": temp,
        "humidity": humidity,
        "battery_voltage": voltage,
        "battery_percentage": battery_percentage,
        "current_mA": current,
        "acceleration_x": acceleration_x,
        "distance": distance
    }

    send_telemetry(telemetry_data)

    lcd.clear()
    lcd.move_to(0, 0)
    lcd.putstr(f"Lat: {latitude[:9]} Temp: {temp}C")
    lcd.move_to(0, 1)
    lcd.putstr(f"Lon: {longitude[:9]}")
    lcd.move_to(0, 2)
    lcd.putstr(f"Speed: {speed} km/h")
    lcd.move_to(0, 3)
    lcd.putstr(f"Course: {course[:4]} Batt: {int(battery_percentage)}%")

    sleep(2)
