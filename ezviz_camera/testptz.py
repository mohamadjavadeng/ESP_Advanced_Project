from pyezvizapi import EzvizClient, EzvizCamera
from pyezvizapi.constants import DeviceSwitchType

client = EzvizClient("mohamadjavadarab92@gmail.com", "Am@n2024", "apiieu.ezvizlife.com")
client.login()                      # add sms_code=... if your account uses MFA

cam = EzvizCamera(client, "BF9010198")

# White spotlight (the H8c's two built-in lights) on / off
cam.set_switch(DeviceSwitchType.LIGHT, True)
cam.set_switch(DeviceSwitchType.LIGHT, False)

# Strobe/flasher (active-defense flashing light)
cam.set_switch(DeviceSwitchType.ALARM_LIGHT, False)

# IR night-vision LEDs, if you want them too
cam.set_switch(DeviceSwitchType.INFRARED_LIGHT, True)
for i in range(5):
    cam.move("down", 1)
# cam.move("left", 1)
client.logout()