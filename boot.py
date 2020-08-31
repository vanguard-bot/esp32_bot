import network
import config


def do_connect():
    import network
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(config.WIFI_SSID, config.WIFI_PASS)
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig()) 


if __name__ == "__main__":
    do_connect()