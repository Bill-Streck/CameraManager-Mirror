import time
import subprocess

process_str = "ffplay -fflags nobuffer -flags low_delay -strict experimental -analyzeduration 0 -probesize 32 -sync ext -framedrop -f mpegts udp://192.168.1.69:9999"
proc = subprocess.Popen(process_str.split())

time.sleep(900)

proc.kill()
    