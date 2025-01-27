import connect 


wlan = connect.do_connect()
if wlan and wlan.isconnected():
    print("Forbindet til WiFI")
    print("IP:", wlan.ifconfig())
else:
    print("Wifi kunne ikke tilsluttes")