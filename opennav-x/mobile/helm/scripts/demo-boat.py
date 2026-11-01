"""Simulated boat for the demo screens (src/demo/*.jpg): NMEA 0183 over TCP on 127.0.0.1:PORT.

Usage: python3 demo-boat.py PORT LAT LON   (the demo: 10611 57.862 11.52, off Marstrand)
SKAGER, in a disposable test profile (tools/prepare-test-profile.py), takes it as a TCP
client connection; the screens were captured at 1600x1000 and saved as JPEG (quality 82).
"""
import math, socket, sys, threading, time
PORT = int(sys.argv[1]); LAT0, LON0 = float(sys.argv[2]), float(sys.argv[3])
def s(body, d='$'):
    c = 0
    for b in body.encode(): c ^= b
    return f'{d}{body}*{c:02X}\r\n'.encode()
def ll(lat, lon):
    a, o = abs(lat), abs(lon)
    return (f'{int(a):02d}{(a-int(a))*60:07.4f}', 'N' if lat >= 0 else 'S', f'{int(o):03d}{(o-int(o))*60:07.4f}', 'E' if lon >= 0 else 'W')
srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); srv.bind(('127.0.0.1', PORT)); srv.listen(2)
start = time.time()
def serve(conn):
    try:
        while True:
            t = time.time() - start
            cog = 300 + 6 * math.sin(t / 40); sog = 5.8 + 0.4 * math.sin(t / 23)
            d = sog * t / 3600 / 60
            lat = LAT0 + d * math.cos(math.radians(cog)); lon = LON0 + d * math.sin(math.radians(cog)) / math.cos(math.radians(LAT0))
            la, ns, lo, ew = ll(lat, lon); g = time.gmtime()
            hms = time.strftime('%H%M%S', g) + '.00'; dmy = time.strftime('%d%m%y', g)
            awa = 42 + 4 * math.sin(t / 17); aws = 14.5 + 1.5 * math.sin(t / 11)
            for m in [s(f'GPRMC,{hms},A,{la},{ns},{lo},{ew},{sog:.1f},{cog:.1f},{dmy},,,A'),
                      s(f'GPGGA,{hms},{la},{ns},{lo},{ew},1,09,0.9,4.0,M,,M,,'),
                      s(f'HCHDT,{(cog-3)%360:.1f},T'), s(f'VWVHW,,T,,M,{sog-0.3:.1f},N,,K'),
                      s(f'SDDPT,{24+6*math.sin(t/30):.1f},0.0'), s(f'WIMWV,{awa:.1f},R,{aws:.1f},N,A'),
                      s(f'YXMTW,{12.4:.1f},C')]:
                conn.sendall(m)
            time.sleep(1)
    except OSError: pass
while True:
    c, _ = srv.accept(); threading.Thread(target=serve, args=(c,), daemon=True).start()
