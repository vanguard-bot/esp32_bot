import ujson as json
import utime as time
from lib import umqtt
from lib import adafruit_gps
from lib.class_imu import MPU6050
from machine import UART, I2C
from lib.utilities import logging_setup_aws, create_message

"""
import upip; upip.install("micropython-intertools)
"""

logger = logging_setup_aws()



def sub_cb(topic, msg):
    result = json.loads(msg)
    result['topic'] = topic
    return result

def isr(pin):
    print("Interrupt!")

def read_record(path):
    with open(path, "r") as f:
        return f.read()

def gps_setup():
    # this uses the UART_1 default pins for TXD and RXD (``P3`` and ``P4``)
    uart = UART(1, bits=8, baudrate=9600)
    # Create a GPS module instance.
    gps = adafruit_gps.GPS(uart)
    # Turn on the basic GGA and RMC info (what you typically want)
    gps.send_command('PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    # Set update rate to once a second (1hz) which is what you typically want.
    gps.send_command('PMTK220,1000')
    gps.update()
    return gps

def connect_mqtt():
    certs = {
            "keyfile":'/cert/a11c050b22-private.pem.key',
            "certfile":'/cert/a11c050b22-certificate.pem.crt',
            "ca_certs":'/cert/AmazonRootCA1.pem'
    }

    for k, v in certs.items():
        certs[k] = read_record(v)
    mqtt = umqtt.MQTTClient(
        client_id=b"esp32bot",  # Name of device
        server=b'a1ca4s6bo9e8zk-ats.iot.us-east-1.amazonaws.com',  # API Endpoint
        ssl=True,
        ssl_params={"key":certs['keyfile'], "cert":certs['certfile'], "server_side":False}  # ssl_params=certs
    )
    message = json.dumps({"Message": "Hello World!"})
    mqtt.connect()
    mqtt.set_callback(sub_cb)
    # mqtt.publish("test/pycom", message)
    # mqtt.disconnect()
    return mqtt


def main():
    # instantiate the i2c interface on esp32 (pins 21,22 for wroom32 variants)
    i2c = I2C(0)  # create on bus 0
    i2c = I2C(0, I2C.MASTER)  # create and init as a master
    i2c = I2C(0, pins=('P10', 'P11'))  # create and use non-default PIN assignments (P10=SDA, P11=SCL)
    i2c.init(I2C.MASTER, baudrate=40000)  # init as a master
    mqtt = connect_mqtt()
    gps = gps_setup()
    seconds = 60
    heartbeat_timer = seconds*10**6 # 60 scconds
    time_sync = time.ticks_us()
    gs1 = MPU6050(i2c=i2c, mpu6050_addr=104)
    gs2 = MPU6050(i2c=i2c, mpu6050_addr=105)
    while True:
        xy_rotation = imu.get_position()
        gps.update()
        if any(abs(v) > 25 for v in xy_rotation.values()):
            dump_msg = create_message(xy_rotation, gps, dumping=True)
            mqtt.publish("test/pycom", dump_msg)
            time.sleep(15)
        elif time.ticks_us() > (time_sync + heartbeat_timer):
            heartbeat_msg = create_message(xy_rotation, gps)
            mqtt.publish("test/pycom", heartbeat_msg)
            time_sync = time.ticks_us()
        time.sleep(0.2)


if __name__ == "__main__":
    mqtt = connect_mqtt()
    message = json.dumps({"Message": "Hello World!"})
    mqtt.publish("test/testing", message)
    mqtt.subscribe("test/testing")
    for _ in range(50):
        sub_mess = mqtt.check_msg()
        if not isinstance(sub_mess, type(None)):
            print(sub_mess)
            updater = sub_mess
        time.sleep(1)

    # import main_publish
    # for _ in range(2):
    #     try:
    #         mqtt = connect_mqtt()
    #     except Exception as e:
    #         logger.exc(e, "Fail to connect to MQTT")
    #         time.sleep(0.2)
    #         continue
    #     break
    # mqtt.publish("test/pycom", "Hello World!")