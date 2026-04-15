@echo off
:: Wait 30 seconds to let Windows fully boot and try to connect first
timeout /t 30
:: Disconnect from current Wi-Fi
netsh wlan disconnect
:: Wait 5 seconds
timeout /t 5
:: Reconnect specifically to eduroam (Make sure the name matches exactly)
netsh wlan connect name="eduroam"
exit