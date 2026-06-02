import serial
from pyubx2 import UBXReader
import time
import socket
from pymavlink import mavutil

gps_port = '/dev/ttyACM0'
laptop_ip = '192.168.2.1' # Laptop IPv4 address
udp_port = 14550          # QGC port

try:
    stream_gps = serial.Serial(gps_port, baudrate=9600, timeout=1)
    ubr = UBXReader(stream_gps, quitonerror=0)
    
    # Direct MAVLink connection to laptop
    mav_conn = mavutil.mavlink_connection(f'udpout:{laptop_ip}:{udp_port}', source_system=1, source_component=220)
    
    print(f"Injecting HIL_GPS MAVLink packets directly to {laptop_ip}:{udp_port}...")
    
    while True:
        raw_data, msg = ubr.read()
        if msg and hasattr(msg, 'identity') and msg.identity == 'NAV-PVT':
            lat_raw = msg.lat
            lon_raw = msg.lon
            alt = msg.height / 1000.0
            fix = msg.fixType
            satellites = msg.numSV
            
            # Proven latitude correction for Beersheba
            if abs(lat_raw) < 1.0:
                lat_deg = lat_raw * 1e7
                lon_deg = lon_raw * 1e7
            else:
                lat_deg = lat_raw / 1e7 if abs(lat_raw) > 90 else lat_raw
                lon_deg = lon_raw / 1e7 if abs(lon_raw) > 180 else lon_raw

            lat_int = int(lat_deg * 1e7)
            lon_int = int(lon_deg * 1e7)
            
            # Build HIL_GPS message
            hil_msg = mav_conn.mav.hil_gps_encode(
                time_usec=int(time.time() * 1000000), 
                fix_type=fix,                         
                lat=lat_int,                          
                lon=lon_int,                          
                alt=int(alt * 1000),                  
                eph=100,                              
                epv=100,                              
                vel=0,                                
                vn=0, ve=0, vd=0,                     
                cog=0,                                
                satellites_visible=satellites         
            )
            
            # Use write() instead of send() for API compatibility
            mav_conn.write(hil_msg.pack(mav_conn.mav))
            
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'stream_gps' in locals(): stream_gps.close()
