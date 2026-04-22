
import serial
import csv
import time
import sys

PORT     = 'COM11'   #whichever port available 
BAUD     = 115200
DURATION = 30
FILENAME = f"gait_fsr_{int(time.time())}.csv"

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    print(f"Connected to {PORT}")
except Exception as e:
    print(f"Error: {e}"); sys.exit()

print(f"Logging for {DURATION}s  →  {FILENAME}")
print("Walk normally. Ctrl+C to stop early.\n")

with open(FILENAME, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['Timestamp', 'Angle', 'Velocity', 'FSR', 'Stage'])

    start = time.time()
    try:
        while True:
            if DURATION and (time.time() - start > DURATION):
                print("\nTime limit reached.")
                break
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line or line.startswith('Timestamp'):
                continue
            parts = line.split(',')
            if len(parts) == 5:
                writer.writerow(parts)
                stage_names = {'1':'Heel Strike','2':'Mid Stance',
                               '3':'Push Off','4':'Swing'}
                name = stage_names.get(parts[4], parts[4])
                print(f"  t={parts[0]}  angle={parts[1]}°  "
                      f"fsr={parts[3]}  stage={name}")
            else:
                print(f"  [skip] {line}")
    except KeyboardInterrupt:
        print("\nStopped.")

ser.close()
print(f"Saved: {FILENAME}")