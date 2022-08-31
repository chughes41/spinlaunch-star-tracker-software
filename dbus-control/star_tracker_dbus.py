import dbus
import time

bus = dbus.SystemBus()

dbus_object = bus.get_object("org.OreSat.StarTracker", "/org/OreSat/StarTracker")

startracker = dbus.Interface(dbus_object, "org.OreSat.StarTracker")

print(startracker.Capture())
print(startracker.ChangeState(0))
time.sleep(2)





