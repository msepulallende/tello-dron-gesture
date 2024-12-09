from djitellopy import Tello
tl= Tello()
tl.connect()
print(tl.get_battery())