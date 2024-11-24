from dronekit import connect, VehicleMode
import time

vehicle = connect('/dev/serial0', baud=912600, wait_ready=True)

latest_rc_channels = None

@vehicle.on_message("RC_CHANNELS")
def rc_channel_listener(vehicle, name, message):
	global latest_rc_channels
	latest_rc_channels = message

def get_rc_channel_value(channel_number):
	global latest_rc_channels
	if latest_rc_channels is None:
		return None
	channel_value = getattr(latest_rc_channels, f"chan{channel_number}_raw", None)
	return channel_value

def radiocontrol():
	if int(get_rc_channel_value(6)) >= 1800:
		print('Giving back control to radio')
		return False
	else:
		return True

while radiocontrol() is True: 
	time.sleep(1)
	print(get_rc_channel_value(6))

