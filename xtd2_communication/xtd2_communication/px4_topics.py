"""PX4 DDS topic names for matching message definitions, including 1.17."""


def px4_topic(direction, name, message_class, namespace=''):
    version = getattr(message_class, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    prefix = namespace.strip('/')
    return f'/{prefix + "/" if prefix else ""}fmu/{direction}/{name}{suffix}'
