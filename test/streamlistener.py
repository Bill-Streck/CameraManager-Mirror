import time
import subprocess

BASE = 53838
# BASE = 33838

def port_from_camera(cid:int):
    return str(BASE + cid)

process_str = "ffplay -fflags nobuffer -flags low_delay " + \
"-strict experimental -analyzeduration 0 -probesize 32 -sync ext " + \
"-framedrop -f mpegts udp://127.0.0.1:" + port_from_camera(3)
proc = subprocess.Popen(process_str.split())

time.sleep(900)

proc.kill()
    