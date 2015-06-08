from gattlib import GATTRequester
import time

# Send: 0x0035 for left
# Send: 0x0039 for right
# Send: 0x0029 for frequency

req = GATTRequester("90:59:AF:14:08:E8")

req.write_by_handle(0x0036, str(bytearray([1])))
time.sleep(1);
req.write_by_handle(0x0039, str(bytearray([1])))