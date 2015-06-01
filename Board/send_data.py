from gattlib import GATTRequester
import time

# Send: 0xB2B5 for left
# Send: 0xB2B6 for right

req = GATTRequester("90:59:AF:14:08:E8")

req.write_by_handle(0x0036, str(bytearray([1])))
time.sleep(1);
req.write_by_handle(0x0039, str(bytearray([1])))