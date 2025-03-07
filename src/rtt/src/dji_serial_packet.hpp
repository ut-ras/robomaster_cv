/*****************************************************************************/
/********** !!! WARNING: CODE GENERATED BY TAPROOT. DO NOT EDIT !!! **********/
/*****************************************************************************/

/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DJI_SERIAL_HPP_
#define DJI_SERIAL_HPP_

#include <cstdint>
#include <cstring>

#include "crc.hpp"

/**
 * A serial handler that implements a specific protocol to be used for
 * communicating with the referee system. Also used for our personal
 * communication with the xavier.
 *
 * Extend this class and implement messageReceiveCallback if you
 * want to use this serial protocol on a serial line.
 *
 * Structure of a Serial Message:
 * \rst
 * +-----------------+------------------------------------------------------------+
 * | Byte Number     | Byte Description                                           |
 * +=================+============================================================+
 * | Frame Header                                                                 |
 * +-----------------+------------------------------------------------------------+
 * | 0               | Frame Head Byte (0xA5)                                     |
 * +-----------------+------------------------------------------------------------+
 * | 1               | Frame Data Length, LSB                                     |
 * +-----------------+------------------------------------------------------------+
 * | 2               | Frame Data Length, MSB                                     |
 * +-----------------+------------------------------------------------------------+
 * | 3               | Frame Sequence Number                                      |
 * +-----------------+------------------------------------------------------------+
 * | 4               | CRC8 of the frame, (bytes 0 - 3)                           |
 * +-----------------+------------------------------------------------------------+
 * | 5               | Message Type, LSB                                          |
 * +-----------------+------------------------------------------------------------+
 * | 6               | Message Type, MSB                                          |
 * +-----------------+------------------------------------------------------------+
 * | Body - Data Length bytes                                                     |
 * +-----------------+------------------------------------------------------------+
 * | Message CRC                                                                  |
 * +-----------------+------------------------------------------------------------+
 * | 7 + Data Length | CRC16 of header and frame, LSB (bytes 0 - 6 + Data Length) |
 * +-----------------+------------------------------------------------------------+
 * | 8 + Data Length | CRC16 of header and frame, MSB                             |
 * +-----------------+------------------------------------------------------------+
 * \endrst
 */

static const uint16_t SERIAL_HEAD_BYTE = 0xA5;

struct FrameHeader
{
    uint8_t head_byte = SERIAL_HEAD_BYTE;
    uint8_t data_length_lsb;
    uint8_t data_length_msb;
    uint8_t sequence;
    uint8_t crc8;
    uint8_t message_type_lsb;
    uint8_t message_type_msb;
};

/**
 * A container for storing and sending message over serial.
 */
struct SerialMessage
{
    /**
     * Constructs a SerialMessage. In doing so this constructor configures the message header.
     *
     * @param[in] seq Message sequence number, an optional parameter.
     */
    explicit SerialMessage(uint8_t *data, uint16_t size, uint16_t messageType, uint8_t seq)
    {
        length = sizeof(FrameHeader) + size + sizeof(*crc16_lsb) + sizeof(*crc16_msb);

        buffer = new uint8_t[length];

        FrameHeader *header = reinterpret_cast<FrameHeader *>(buffer);
        crc16_lsb = reinterpret_cast<uint8_t *>(buffer + sizeof(FrameHeader) + size);
        crc16_msb = reinterpret_cast<uint8_t *>(buffer + sizeof(FrameHeader) + size + sizeof(*crc16_lsb));

        header->head_byte = SERIAL_HEAD_BYTE;
        header->data_length_lsb = size & 0xFF;
        header->data_length_msb = (size >> 8) & 0xFF;
        header->sequence = seq;

        header->crc8 = algorithms::calculateCRC8(
            reinterpret_cast<uint8_t *>(header),
            sizeof(FrameHeader) - sizeof(FrameHeader::crc8) - sizeof(FrameHeader::message_type_lsb) - sizeof(FrameHeader::message_type_msb));

        header->message_type_lsb = messageType & 0xFF;
        header->message_type_msb = (messageType >> 8) & 0xFF;

        memcpy(buffer + sizeof(FrameHeader), data, size);

        uint16_t crc16 = algorithms::calculateCRC16(
            buffer,
            length - sizeof(*crc16_lsb) - sizeof(*crc16_msb));

        *crc16_lsb = crc16 & 0xFF;
        *crc16_msb = (crc16 >> 8) & 0xFF;
    }

    uint16_t length;
    uint8_t *buffer;
    uint8_t *crc16_lsb;
    uint8_t *crc16_msb;
};

#endif // DJI_SERIAL_HPP_