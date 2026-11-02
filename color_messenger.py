# color_messenger.py
# Sends the name of a detected tape color ("red", "green" or "blue")
# to another ESP32 using ESP-NOW. Imported by line_follower.py.
#
# ESP-NOW uses the other board's MAC address (not its IP address).
# Run color_receiver.py on the other ESP32: it prints its MAC address
# when it starts. Copy that into PEER_MAC below.

import network
import espnow

# MAC address of the receiving ESP32, e.g. b'\x24\x6f\x28\xaa\xbb\xcc'
#PEER_MAC = b'\x20\x9b\xa9\x74\x72\x60'
PEER_MAC = b'\xff\xff\xff\xff\xff\xff'


_esp = None


def start():
    """Turns on Wi-Fi (needed for ESP-NOW) and registers the peer."""
    global _esp
    # Newer MicroPython uses network.WLAN.IF_STA, older uses network.STA_IF
    if_sta = getattr(network.WLAN, "IF_STA", None)
    if if_sta is None:
        if_sta = network.STA_IF
    sta = network.WLAN(if_sta)
    sta.active(True)
    sta.disconnect()

    _esp = espnow.ESPNow()
    _esp.active(True)
    try:
        _esp.add_peer(PEER_MAC)
    except OSError:
        pass                 # peer already added (e.g. after a soft reset)


def send_color(color):
    """Sends the color name. Never stops the car if sending fails."""
    if _esp is None:
        return
    try:
        # sync=False: don't wait for the receiver to confirm, so the
        # control loop isn't held up
        _esp.send(PEER_MAC, color, False)
    except OSError as err:
        print("ESP-NOW send failed:", err)