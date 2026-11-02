import network
import espnow
import time

# A WLAN interface must be active to send()/recv()
sta = network.WLAN(network.WLAN.IF_STA)
sta.active(True)
sta.disconnect()   # Because ESP8266 auto-connects to last Access Point

e = espnow.ESPNow()
e.active(True)

mac_addresses = []
  
def recv_cb(esp):
    if esp.any():
        mac, msg = esp.irecv(0)  # Read immediate message buffer
        if mac is not None:
            print(mac, msg)
                    
e.irq(recv_cb)
