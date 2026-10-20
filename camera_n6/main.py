# =====================================================================
#  HAB-RECON - CAMERA (Cormorant N6, OpenMV-compatible MicroPython)
# =====================================================================
#  Stand-in detector for the demo: finds RED objects and sends them to the
#  payload ESP32 over UART as one short text line per detection:
#
#      DET,<label>,<confidence 0-100>,<x 0-1000>,<y 0-1000>\n
#
#  x / y are the object's centre in the image (0 = left/top, 1000 = right/bottom).
#  Hold up a red card under the camera -> a "target" appears on the map.
#
#  Later: replace find_blobs() with your neural-network model and send its
#  class name as <label> (person / vehicle / drone / target / object).
#
#  UNTESTED TEMPLATE - check with your friend:
#    * which UART number is wired to the camera's UART port (UART_ID)
#    * that the port is 3.3 V logic before connecting it to the XIAO
#    * newer OpenMV firmware may use the "csi" module instead of "sensor"
# =====================================================================
import sensor
import time
from machine import UART

UART_ID = 1            # TODO: confirm for the Cormorant N6
BAUD = 115200
LABEL = "target"
RED = [(30, 100, 15, 127, 15, 127)]   # LAB colour threshold for red
MIN_INTERVAL_MS = 500                 # don't flood the link

uart = UART(UART_ID, BAUD)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)     # 320x240 is plenty for colour blobs
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)           # needed for stable colour tracking
sensor.set_auto_whitebal(False)

last = time.ticks_ms()
while True:
    img = sensor.snapshot()
    blobs = img.find_blobs(RED, pixels_threshold=200, area_threshold=200, merge=True)
    if not blobs:
        continue
    b = max(blobs, key=lambda x: x.pixels())
    img.draw_rectangle(b.rect(), color=(0, 255, 0))
    img.draw_cross(b.cx(), b.cy(), color=(0, 255, 0))
    if time.ticks_diff(time.ticks_ms(), last) < MIN_INTERVAL_MS:
        continue
    last = time.ticks_ms()
    x = int(b.cx() * 1000 / img.width())
    y = int(b.cy() * 1000 / img.height())
    conf = min(99, int(b.density() * 100))
    uart.write("DET,%s,%d,%d,%d\n" % (LABEL, conf, x, y))
