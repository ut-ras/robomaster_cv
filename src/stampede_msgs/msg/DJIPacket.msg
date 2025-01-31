uint8 FRAME_HEAD_BYTE = 0xA5
uint8 frame_head_byte
uint16 frame_data_length
uint8 frame_sequence_number
uint8 crc8
uint16 message_type
uint8[] body
uint16 crc16