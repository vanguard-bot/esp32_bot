import machine
import network
import time
from ntptime import settime
from MQTT import MQTTClient


def start_deep_sleep(sleep_time=60):
    # check if the device woke from a deep sleep
    if machine.reset_cause() == machine.DEEPSLEEP_RESET:
        print('woke from a deep sleep')

    # put the device to sleep for 60 seconds
    machine.deepsleep(sleep_time * 1000)


def wifi_connect(username, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(username, password)
        count = 0
        while not wlan.isconnected():
            time.sleep(.2)
            if count > 20:
                machine.reset()
    print("Network config:", wlan.ifconfig())
    settime()


def sub_cb(topic, msg):
    print(msg)


def gen_mqtt_client():
    client = MQTTClient("office_plant_1", "io.adafruit.com",
                        user="jmr82986",
                        password="f02c3a96f10f4e0d858c42360396e3a4",
                        port=1883)
    client.set_callback(sub_cb)
    client.connect()
    client.subscribe(topic="jmr82986/feeds/office_plant")
    return client
