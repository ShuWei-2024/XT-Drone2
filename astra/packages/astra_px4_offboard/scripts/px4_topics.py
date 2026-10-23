"""Resolve PX4 DDS topics and shared sensor subscription QoS."""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


PX4_HISTORY_DEPTH = 64


def px4_sensor_history_qos():
    """Retain samples through a brief executor stall without loosening gates.

    At 100 Hz, depth 64 holds about 640 ms. Consumers still enforce their
    measurement-age and timestamp-pair limits before accepting any sample.
    """
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=PX4_HISTORY_DEPTH,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def px4_topic(direction, name, message_class, namespace=''):
    version = getattr(message_class, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    prefix = namespace.strip('/')
    return f'/{prefix + "/" if prefix else ""}fmu/{direction}/{name}{suffix}'
