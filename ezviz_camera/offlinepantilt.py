from onvif import ONVIFCamera
import time

cam = ONVIFCamera(
    "192.168.100.29", 80, "admin", "Aman2026",
    wsdl_dir=r"C:\Users\Asus\AppData\Roaming\Python\Lib\site-packages\wsdl",
    adjust_time=True,          # <-- fixes "sender is not authorized" from clock skew
)

media = cam.create_media_service()
ptz = cam.create_ptz_service()
token = media.GetProfiles()[0].token
print(token)
req = ptz.create_type("ContinuousMove")
req.ProfileToken = token
req.Velocity = {"PanTilt": {"x": 0.5, "y": 0.5}}   # pan right
ptz.ContinuousMove(req)
time.sleep(1.0)
ptz.Stop({"ProfileToken": token})