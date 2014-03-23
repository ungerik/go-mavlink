package mavlink

import (
	"bytes"
	"encoding/binary"
)

var charParserFactory = map[parserState]parserFunc{
	mavlink_parse_state_idle:        parseFramestart,
	mavlink_parse_state_got_stx:     parsePayloadLength,
	mavlink_parse_state_got_length:  parsePacketSeq,
	mavlink_parse_state_got_seq:     parseSysID,
	mavlink_parse_state_got_sysid:   parseCompID,
	mavlink_parse_state_got_compid:  parseMsgID,
	mavlink_parse_state_got_msgid:   parsePayload,
	mavlink_parse_state_got_payload: parseCrc1,
	mavlink_parse_state_got_crc1:    parseCrc2,
}

type parserFunc func(byte, *MavPacketInternal) (parserState, error)
type PacketSequenceFunc func() uint8

/**
 * Simply incr the packet sequence
 */
type MavPacketManager struct {
	get PacketSequenceFunc
}

func getPacketSeqGenerator() PacketSequenceFunc {
	packetSequence := uint8(0)

	return func() (ret uint8) {
		ret = packetSequence
		if packetSequence == 0xff {
			packetSequence = 0
		}
		packetSequence++
		return
	}
}

/**
 * Parser
 */

func parseFramestart(c byte, pInternal *MavPacketInternal) (parserState, error) {
	if c == frameStart {
		pInternal.rawBuffer.Reset()
		pInternal.rawBuffer.WriteByte(c)
		pInternal.packet = new(MavPacket)
		pInternal.packet.crcInit()
		pInternal.packet.Header.FrameStart = frameStart
		return mavlink_parse_state_got_stx, nil
	}
	return mavlink_parse_state_idle, ErrInvalidStartframe(c)
}

func parsePayloadLength(c byte, pInternal *MavPacketInternal) (parserState, error) {
	if c >= 0 && c <= mav_max_payload_len {
		pInternal.rawBuffer.WriteByte(c)
		pInternal.packet.Header.PayloadLength = c
		pInternal.packet.crcAccumulate(c)
		return mavlink_parse_state_got_length, nil
	}
	return mavlink_parse_state_idle, ErrInvalidPayloadLength(c)
}

func parsePacketSeq(c byte, pInternal *MavPacketInternal) (parserState, error) {
	pInternal.rawBuffer.WriteByte(c)
	pInternal.packet.Header.PacketSequence = c
	pInternal.packet.crcAccumulate(c)
	return mavlink_parse_state_got_seq, nil
}

func parseSysID(c byte, pInternal *MavPacketInternal) (parserState, error) {
	pInternal.rawBuffer.WriteByte(c)
	pInternal.packet.Header.SystemID = c
	pInternal.packet.crcAccumulate(c)
	return mavlink_parse_state_got_sysid, nil
}

func parseCompID(c byte, pInternal *MavPacketInternal) (parserState, error) {
	pInternal.rawBuffer.WriteByte(c)
	pInternal.packet.Header.ComponentID = c
	pInternal.packet.crcAccumulate(c)
	return mavlink_parse_state_got_compid, nil
}

func parseMsgID(c byte, pInternal *MavPacketInternal) (parserState, error) {
	pInternal.rawBuffer.WriteByte(c)
	pInternal.packet.Header.MessageID = c
	pInternal.packet.crcAccumulate(c)

	if pInternal.packet.Header.PayloadLength == 0 {
		return mavlink_parse_state_got_payload, nil
	}

	if msgGenerator, ok := messageFactory[c]; ok {
		pInternal.packet.Msg = msgGenerator()
		if pInternal.packet.Header.PayloadLength != pInternal.packet.Msg.Size() {
			return mavlink_parse_state_idle, ErrInvalidPayloadLength(pInternal.packet.Header.PayloadLength)
		} else {
			return mavlink_parse_state_got_msgid, nil
		}
	}
	return mavlink_parse_state_idle, ErrUnknownMessageID(c)
}

func parsePayload(c byte, pInternal *MavPacketInternal) (parserState, error) {
	pInternal.rawBuffer.WriteByte(c)
	pInternal.packet.crcAccumulate(c)

	if pInternal.rawBuffer.Len()-int(pInternal.packet.Header.Size()) == int(pInternal.packet.Msg.Size()) {
		buf := bytes.NewBuffer(pInternal.rawBuffer.Bytes()[pInternal.packet.Header.Size():])
		if err := binary.Read(buf, binary.LittleEndian, pInternal.packet.Msg); err != nil {
			return mavlink_parse_state_got_msgid, err
		}
		return mavlink_parse_state_got_payload, nil
	}
	return mavlink_parse_state_got_msgid, nil
}

func parseCrc1(c byte, pInternal *MavPacketInternal) (parserState, error) {
	pInternal.rawBuffer.WriteByte(c)

	if mavlink_crc_extra_enabled {
		pInternal.packet.crcAccumulate(messageCrcs[pInternal.packet.Msg.ID()])
	}
	if c == uint8(pInternal.packet.Checksum&0xFF) {
		return mavlink_parse_state_got_crc1, nil
	} else if c == frameStart {
		return parseFramestart(c, pInternal)
	}
	return mavlink_parse_state_idle, ErrInvalidChecksum(c)
}

func parseCrc2(c byte, pInternal *MavPacketInternal) (parserState, error) {
	pInternal.rawBuffer.WriteByte(c)

	if c == uint8((pInternal.packet.Checksum >> 8)) {
		return mavlink_parse_state_got_packet, nil
	} else if c == frameStart {
		return parseFramestart(c, pInternal)
	}
	return mavlink_parse_state_idle, ErrInvalidChecksum(c)
}
