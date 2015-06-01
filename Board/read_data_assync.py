from gattlib import GATTRequester, GATTResponse
import time

req = GATTRequester("90:59:AF:14:08:E8")
response = GATTResponse()

req.read_by_handle_async(0x15, response)
while not response.received():
    time.sleep(0.1)

steps = response.received()[0]
print steps.encode('hex')