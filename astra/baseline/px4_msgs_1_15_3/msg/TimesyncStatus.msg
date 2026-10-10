uint64 timestamp

uint8 SOURCE_PROTOCOL_UNKNOWN = 0
uint8 SOURCE_PROTOCOL_MAVLINK = 1
uint8 SOURCE_PROTOCOL_DDS = 2
uint8 source_protocol

uint64 remote_timestamp
int64 observed_offset
int64 estimated_offset
uint32 round_trip_time
