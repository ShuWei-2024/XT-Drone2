"""Local SITL GCS heartbeat only: no commands or setpoints."""
import time
from pymavlink import mavutil


def main():
    connection = mavutil.mavlink_connection('udpout:127.0.0.1:18570',
                                           source_system=255, source_component=190)
    try:
        while True:
            connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE)
            time.sleep(1.)
    finally:
        connection.close()


if __name__ == '__main__':
    main()
