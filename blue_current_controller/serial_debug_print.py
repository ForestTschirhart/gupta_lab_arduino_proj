import serial, sys

PORT = "/dev/ttyACM1"   # adjust to your device
BAUD = 250000

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"Error opening {PORT}: {e}", file=sys.stderr)
    sys.exit(1)

print(f"Listening on {PORT} @ {BAUD}...")
try:
    while True:
        line = ser.readline()
        if not line:
            continue
        print(line.decode("utf-8", errors="replace").rstrip())
except KeyboardInterrupt:
    pass
finally:
    ser.close()
