# Import mavutil
from pymavlink import mavutil
import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM)
servoPIN = 17
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)#Button to GPIO23
GPIO.setup(servoPIN, GPIO.OUT)
p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()
# Choose a mode

# Set all rc chans pwm to 1500 before arming:
data = [ 1500 ] * 8
master.mav.rc_channels_override_send(
	master.target_system, master.target_component, *data)

p.start(2.5)
#buka
p.ChangeDutyCycle(10)
time.sleep(1)
##tutup
p.ChangeDutyCycle(5)
time.sleep(0.7)
p.stop()

try:
    while True:
         button_state = GPIO.input(23)
         if button_state == False:
             ##ganti mode
             mode = 'ALT_HOLD'

             # Check if mode is available
             if mode not in master.mode_mapping():
                 print('Unknown mode : {}'.format(mode))
                 print('Try:', list(master.mode_mapping().keys()))
                 exit(1)

             # Get mode ID
             mode_id = master.mode_mapping()[mode]
             master.mav.set_mode_send(
                 master.target_system,
                 mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                 mode_id)

             ####arming ###
             master.mav.manual_control_send(
                 master.target_system,
                 0,
                 0,
                 0,
                 0,
                 16)

             print('Activated')
             time.sleep(1)
             master.mav.command_long_send(
                 master.target_system,
                 master.target_component,
                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                 1,
                 1, 0, 0, 0, 0, 0, 0)

             ##RC thr
             data[2] = 1400  # set some throttle turun
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(3)  # turun 1 meter
             data[2] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju
             data[4] = 1600  # set maju 11m
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(42)
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##rc putar kiri
             data[3] = 1400  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju
             data[4] = 1600  # set maju 11m
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2)##1.5m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)


             ##Realease Servo OPEN
             p.start(2.5)
             ##buka
             p.ChangeDutyCycle(10)
             time.sleep(1)
             ##tutu
             p.ChangeDutyCycle(5)
             time.sleep(0.7)
             p.stop()
             ##rc putar
             data[3] = 1600  # set putar 180
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##slayer di akhir
             ##maju horizon  (1)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(28)  ##3m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##rc putar kanan (2)
             data[3] = 1600  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju vertical (3)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(3)  ##1.25m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##rc putar kanan (4)
             data[3] = 1600  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju horizon (5)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(3)  ##1.5m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##rc putar kiri (6)
             data[3] = 1400  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju vartical (7)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(3)  ##1.5m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##rc putar kiri (8)
             data[3] = 1400  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju horizon (9)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(6)  ##3m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##rc putar kanan (10)
             data[3] = 1600  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju vartical (11)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(3)  ##1.5m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##rc putar kanan (12)
             data[3] = 1600  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju horizon (13)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(6)  ##3m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##kiri (14)
             data[3] = 1400  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju vertical(15)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(3)  ##1.5m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##kiri (16)
             data[3] = 1400  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju 6m (17)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(6)  ##3m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##rc putar kanan (18)
             data[3] = 1600  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju vartical (19)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(3)  ##1.5m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##rc putar kanan (20)
             data[3] = 1600  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju horizon (21)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(6)  ##3m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             ##kiri (22)
             data[3] = 1400  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             ##maju vertical(23)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(3)  ##1.5m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             ##kiri (24)
             data[3] = 1400  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju 6m (25)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(6)  ##3m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##rc putar kanan (26)
             data[3] = 1600  # set putar 90
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(2.5)
             data[3] = 1500  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##maju vartical (27)
             data[4] = 1600  # set maju
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(3)  ##1.5m
             data[4] = 1500
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             ##gantimode
             mode = 'STABILIZE'

             # Check if mode is available
             if mode not in master.mode_mapping():
                 print('Unknown mode : {}'.format(mode))
                 print('Try:', list(master.mode_mapping().keys()))
                 exit(1)

             # Get mode ID
             mode_id = master.mode_mapping()[mode]
             master.mav.set_mode_send(
                 master.target_system,
                 mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                 mode_id)

             ##RC thr ending surfacing
             data[2] = 1600  # set some throttle na
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
             time.sleep(3)  # naik 1 meter
             data[2] = 1570  # set some throttle
             master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)

             break

         else:
             ##nyala
             print('DeActivated')
             time.sleep(1)
             master.mav.command_long_send(
                 master.target_system,
                 master.target_component,
                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                 1,
                 0, 0, 0, 0, 0, 0, 0)
except:
    p.stop()
