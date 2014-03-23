package mavlink

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
)

type parserState uint8
type ParseCharFunc func(byte) (packet *MavPacket, err error)

const (
	mavlink_crc_extra_enabled       = true
	frameStart                      = uint8(0xFE)
	mavlink_parse_state_uninit      = 0
	mavlink_parse_state_idle        = 1
	mavlink_parse_state_got_stx     = 2
	mavlink_parse_state_got_length  = 3
	mavlink_parse_state_got_seq     = 4
	mavlink_parse_state_got_sysid   = 5
	mavlink_parse_state_got_compid  = 6
	mavlink_parse_state_got_msgid   = 7
	mavlink_parse_state_got_payload = 8
	mavlink_parse_state_got_crc1    = 9
	mavlink_parse_state_got_packet  = 10
)

var (
	packetSeqGenerator = getPacketSeqGenerator()
)

type ErrUnknownMessageID uint8

func (self ErrUnknownMessageID) Error() string {
	return fmt.Sprintf("Unknow message ID %d", self)
}

type ErrInvalidChecksum uint16

func (self ErrInvalidChecksum) Error() string {
	return fmt.Sprintf("Invalid checksum %d", self)
}

type ErrInvalidStartframe uint8

func (self ErrInvalidStartframe) Error() string {
	return fmt.Sprintf("Invalid start of frame %d", self)
}

type ErrInvalidPayloadLength uint8

func (self ErrInvalidPayloadLength) Error() string {
	return fmt.Sprintf("Message ID and PayloadLength (%d) don't match", self)
}

// Create a MavPacket, and write it on the writer
func Send(writer io.Writer, systemID, componentID uint8, message Message) error {
	packet, err := CreatePacket(systemID, componentID, message)
	if err != nil {
		return err
	}

	return binary.Write(writer, binary.LittleEndian, packet.Bytes())
}

// Create a MavPacket then compute the checksum.
// TODO : compute the checksum on the go
func CreatePacket(systemID, componentID uint8, message Message) (*MavPacket, error) {
	packet := &MavPacket{
		Header: MavHeader{
			FrameStart:     frameStart,
			PayloadLength:  message.Size(),
			PacketSequence: packetSeqGenerator(),
			SystemID:       systemID,
			ComponentID:    componentID,
			MessageID:      message.ID(),
		},
		Msg: message,
	}

	packet.computeChecksum()
	return packet, nil // todo : check errors...
}

func checkChecksum(buf *bytes.Buffer, packet *MavPacket) error {
	var streamChecksum uint16

	err := binary.Read(buf, binary.LittleEndian, &streamChecksum)
	if err != nil {
		return err
	}
	if streamChecksum != packet.computeChecksum() {
		return ErrInvalidChecksum(streamChecksum)
	}
	return nil
}

func GetMavParser() ParseCharFunc {
	var step parserState
	var pInternal *MavPacketInternal = new(MavPacketInternal)

	pInternal.packet = new(MavPacket)
	step = mavlink_parse_state_idle

	return func(c byte) (_ *MavPacket, err error) {
		if charParser, ok := charParserFactory[step]; ok {
			step, err = charParser(c, pInternal)
			if err != nil {
				pInternal.rawBuffer.Reset()
				return nil, err
			}
		}
		if step == mavlink_parse_state_got_packet {
			pInternal.rawBuffer.Reset()
			step = mavlink_parse_state_idle
			return pInternal.packet, nil
		}
		return nil, nil
	}
}
