package mavlink

import (
	"bytes"
	"encoding/binary"
)

const (
	mav_max_payload_len   = 255
	mav_max_packet_lenght = 263 // sizeof(message) + sizeof(header) + sizeof(checksum)
	x25InitCrc            = uint16(0xffff)
	x25ValidateCrc        = uint16(0xf0b8)
)

var (
	messageCrcs = []byte{50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 214, 223, 141, 33, 15, 3, 100, 24, 239, 238, 30, 240, 183, 130, 130, 0, 148, 21, 0, 243, 124, 0, 0, 0, 20, 0, 152, 143, 0, 0, 127, 106, 0, 0, 0, 0, 0, 0, 0, 231, 183, 63, 54, 0, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 0, 0, 0, 0, 235, 93, 124, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 42, 241, 15, 134, 219, 208, 188, 84, 22, 19, 21, 134, 0, 78, 68, 189, 127, 111, 21, 21, 144, 1, 234, 73, 181, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 204, 49, 170, 44, 83, 46, 0}
)

type MavPacketInternal struct {
	packet    *MavPacket
	rawBuffer bytes.Buffer
}

type MavHeader struct {
	FrameStart     uint8
	PayloadLength  uint8
	PacketSequence uint8
	SystemID       uint8
	ComponentID    uint8
	MessageID      uint8
}

type MavPacket struct {
	Header   MavHeader
	Msg      Message
	Checksum uint16
}

func (this *MavPacketInternal) computeChecksum() uint16 {
	this.packet.crcInit()
	/* 1 represents indexOf header.PayloadLength */
	for _, v := range this.rawBuffer.Bytes()[1 : this.packet.Header.Size()+this.packet.Msg.Size()] {
		this.packet.crcAccumulate(v)
	}

	if mavlink_crc_extra_enabled {
		this.packet.crcAccumulate(messageCrcs[this.packet.Msg.ID()])
	}
	return this.packet.Checksum
}

func (this *MavHeader) Size() uint8 {
	return 6
}

/* not used for now */
func (this *MavPacket) crcAccumulateBuffer() {
	b := make([]byte, 2)
	binary.LittleEndian.PutUint16(b, this.Checksum)
	for _, v := range b {
		this.crcAccumulate(v)
	}
}

/* Accumulate one byte of data into the CRC */
func (this *MavPacket) crcAccumulate(data uint8) {
	var tmp uint8

	tmp = data ^ uint8(this.Checksum&0xff)
	tmp ^= (tmp << 4)
	this.Checksum = (this.Checksum >> 8) ^ (uint16(tmp) << 8) ^ (uint16(tmp) << 3) ^ (uint16(tmp) >> 4)
}

func (this *MavPacket) crcInit() {
	this.Checksum = x25InitCrc
}

func (this *MavPacket) computeChecksum() uint16 {
	this.crcInit()
	/* 1 represents indexOf header.PayloadLength */

	for _, v := range this.Bytes()[1 : this.Header.Size()+this.Msg.Size()] {
		this.crcAccumulate(v)
	}

	if mavlink_crc_extra_enabled {
		this.crcAccumulate(messageCrcs[this.Msg.ID()])
	}
	return this.Checksum
}

/**
 * Need to be removed.
 * Now used for the checksum computing
 */

func (this *MavPacket) Bytes() []byte {
	var r bytes.Buffer
	var err error

	if err = binary.Write(&r, binary.LittleEndian, this.Header); err != nil {
		return []byte{}
	}

	if err = binary.Write(&r, binary.LittleEndian, this.Msg); err != nil {
		return []byte{}
	}
	if err = binary.Write(&r, binary.LittleEndian, this.Checksum); err != nil {
		return []byte{}
	}
	return r.Bytes()
}
