# Imports - built-ins
import time
import threading
import logging
from enum import Enum

# Imports - external
from pydbus import SystemBus
from gi.repository import GLib
from systemd import journal

# Imports - custom modules
from snapper import Snapper


snapper = Snapper(0)
time.sleep(2)
snapper.capture_snap(5)
snapper.capture_solve(51) 
#snapper.capture_snap(5)
time.sleep(2)
#snapper.capture_solve(5)
#time.sleep(2)
#snapper.stop()
#time.sleep(2)
#snapper.start()
#time.sleep(1)
#snapper.stop()
#time.sleep(1)#
#snapper.restart()
#time.sleep(1)
#snapper.stop()
