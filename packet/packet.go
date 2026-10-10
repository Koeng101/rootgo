// Package packet implements the 20-byte frame format used by iRobot Root
// robots over BLE.
//
// Every frame is laid out as:
//
//	byte 0      device
//	byte 1      command
//	byte 2      packet ID
//	bytes 3-18  payload (big endian, zero padded)
//	byte 19     CRC-8 (poly 0x07, init 0x00) over bytes 0-18
//
// Note that the protocol documentation numbers fields by frame byte, so a
// field documented at "Bytes 3:6" lives at Payload[0:4].
package packet

import (
	"encoding/hex"
	"errors"
	"fmt"
)

const (
	// Size is the length of every frame on the wire.
	Size = 20
	// PayloadSize is the number of payload bytes in a frame.
	PayloadSize = 16
)

var (
	ErrSize = errors.New("packet: frame is not 20 bytes")
	ErrCRC  = errors.New("packet: CRC mismatch")
)

// Packet is a decoded frame.
type Packet struct {
	Dev     uint8
	Cmd     uint8
	ID      uint8
	Payload [PayloadSize]byte
}

// New builds a packet, copying payload into the zero-padded payload field.
// It panics if payload is longer than PayloadSize, which is always a
// programming error.
func New(dev, cmd, id uint8, payload []byte) Packet {
	if len(payload) > PayloadSize {
		panic(fmt.Sprintf("packet: payload is %d bytes, max %d", len(payload), PayloadSize))
	}
	p := Packet{Dev: dev, Cmd: cmd, ID: id}
	copy(p.Payload[:], payload)
	return p
}

// Bytes encodes the packet into a frame, including its CRC.
func (p Packet) Bytes() [Size]byte {
	var b [Size]byte
	b[0], b[1], b[2] = p.Dev, p.Cmd, p.ID
	copy(b[3:Size-1], p.Payload[:])
	b[Size-1] = CRC8(b[:Size-1])
	return b
}

// Parse decodes a frame, verifying its length and CRC.
func Parse(b []byte) (Packet, error) {
	if len(b) != Size {
		return Packet{}, ErrSize
	}
	if CRC8(b[:Size-1]) != b[Size-1] {
		return Packet{}, ErrCRC
	}
	p := Packet{Dev: b[0], Cmd: b[1], ID: b[2]}
	copy(p.Payload[:], b[3:Size-1])
	return p, nil
}

func (p Packet) String() string {
	return fmt.Sprintf("Packet[%d %d %d] %s", p.Dev, p.Cmd, p.ID, hex.EncodeToString(p.Payload[:]))
}

// CRC8 computes the robot's checksum: CRC-8 with polynomial 0x07, initial
// value 0, no reflection and no final XOR.
func CRC8(data []byte) byte {
	var crc byte
	for _, c := range data {
		crc ^= c
		for range 8 {
			if crc&0x80 != 0 {
				crc = crc<<1 ^ 0x07
			} else {
				crc <<= 1
			}
		}
	}
	return crc
}
