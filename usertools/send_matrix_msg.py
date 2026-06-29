from scapy.all import Ether, sendp
import struct

IFACE = "en5"                         # sender interface
DST_MAC = "02:00:00:00:00:01"          # MAC printed by matrix_rx
ETHERTYPE_MATRIX = 0x88BD

rows = 2
cols = 2
values = [1.0, 2.0, 3.0, 4.0]

payload = b"MX01"
payload += struct.pack("!H", rows)
payload += struct.pack("!H", cols)
payload += struct.pack("!B", 1)        # type 1 = float32
payload += b"\x00\x00\x00"             # padding

# matrix_rx memcpy()s floats directly, so send floats in sender CPU byte order
payload += b"".join(struct.pack("f", v) for v in values)

frame = Ether(dst=DST_MAC, type=ETHERTYPE_MATRIX) / payload

sendp(frame, iface=IFACE, count=1, verbose=True)
