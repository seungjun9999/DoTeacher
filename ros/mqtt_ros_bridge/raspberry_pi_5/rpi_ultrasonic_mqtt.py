import paho.mqtt.client as mqtt
import json
import time
from gpiozero import DistanceSensor

# 초음파 센서 설정
sensor = DistanceSensor(echo=17, trigger=4)

# MQTT 설정
broker_address = "<RPI의 IP주소>"  # MQTT 브로커 IP 주소
topic = "sensor/ultrasonic"

def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected with result code "+str(rc))

def on_message(client, userdata, msg):
    print(f"Received message: {msg.topic} {msg.payload}")

# MQTT 클라이언트 설정
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(broker_address, 1883, 60)
    client.loop_start()

    while True:
        distance = sensor.distance * 100  # cm로 변환
        msg = {"distance": distance}
        client.publish(topic, json.dumps(msg))
        print(f"Published: {distance:.2f} cm")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    client.loop_stop()
    client.disconnect()