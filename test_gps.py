import serial
from pyubx2 import UBXReader

port = '/dev/ttyACM0'
try:
    stream = serial.Serial(port, baudrate=9600, timeout=1)
    ubr = UBXReader(stream, quitonerror=2)
    print("Connecting to GPS... Waiting for messages...")
    
    while True:
        raw_data, msg = ubr.read()
        if msg and hasattr(msg, 'identity') and msg.identity == 'NAV-PVT':
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            fix = msg.fixType # 0=None, 1=Searching, 3=Good 3D lock
            satellites = msg.numSV
            
            print(f"Fix Type: {fix} | Satellites: {satellites} | Lat: {lat} | Lon: {lon}")
except Exception as e:
    print(f"Error: {e}")
