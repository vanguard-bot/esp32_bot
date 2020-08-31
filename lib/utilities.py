import machine
import network
import time
from ntptime import settime
from umqtt.simple import MQTTClient
import logging
import json
from ubinascii import hexlify

def format_datetime(dt):
    t="{year}-{mm}-{dd}T{hh}:{minute}:{ss}".format(year=dt[0],
                                                   mm=dt[1] if dt[1] < 10 else "0{t}".format(dt[1]),
                                                   dd=dt[2] if dt[2] < 10 else "0{t}".format(dt[2]),
                                                   hh=dt[3] if dt[3] < 10 else "0{t}".format(dt[3]),
                                                   minute=dt[4] if dt[4] < 10 else "0{t}".format(dt[4]),
                                                   ss=dt[5] if dt[5] < 10 else "0{t}".format(dt[5]))
    return t

def logging_setup_aws(log_level: int = 10):
    """
    setup logging
    :return: a setup logger
    """
    # setting up logging
    logging.basicConfig(level=log_level)
    logger_ = logging.getLogger()
    logger_.setLevel(log_level)
    logger_.info('starting app...')
    return logger_


def random_int(start=1000, end=2000):
    t = time.ticks_us() % end
    if t >= start:
        return t
    else:
        return random_int(start=start, end=end)

def create_message(gps, status, **kwargs):
    records = {
            "gps": gps,
            "status": status
        }
    if len(list(kwargs.keys())) > 0:
        records.update(kwargs)
    return b"{records}".format(records=json.dumps(records)).replace(b", ", b",")
    

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
