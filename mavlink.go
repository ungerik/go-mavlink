package mavlink

import (
	"encoding/binary"
	"fmt"
	"io"
	// "io/ioutil"
)

var (
	frameStart      = []byte{'\xfe'}
	sequenceCounter uint8
)

type checksum struct {
	writer io.Writer
	sum    uint16
}

func (self *checksum) Write(data []byte) (n int, err error) {
	// todo calc sum

	if self.writer != nil {
		return self.writer.Write(data)
	} else {
		return len(data), nil
	}
}

func Send(writer io.Writer, systemID, componentID uint8, message Message) error {
	_, err := writer.Write(frameStart)
	if err != nil {
		return err
	}
	checksum := checksum{writer: writer}

	header := []byte{message.Size(), sequenceCounter, systemID, componentID, message.ID()}
	_, err = checksum.Write(header)
	if err != nil {
		return err
	}

	err = binary.Write(&checksum, binary.LittleEndian, message)
	if err != nil {
		return err
	}

	return binary.Write(writer, binary.LittleEndian, checksum.sum)
}

type header struct {
	PayloadLength  uint8
	PacketSequence uint8
	SystemID       uint8
	ComponentID    uint8
	MessageID      uint8
}

type ErrUnknownMessageID uint8

func (self ErrUnknownMessageID) Error() string {
	return fmt.Sprintf("Unknow message ID %d", self)
}

type ErrInvalidChecksum uint16

func (self ErrInvalidChecksum) Error() string {
	return fmt.Sprintf("Invalid checksum %d", self)
}

func Receive(reader io.Reader) (msg Message, systemID, componentID uint8, err error) {
	// Read first byte and check if it is 0xfe
	var start [1]byte
	_, err = reader.Read(start[:])
	if err != nil {
		return nil, 0, 0, err
	}
	if start[0] != 0xfe {
		return nil, 0, 0, fmt.Errorf("Invalid start of frame %x", start)
	}

	// Create a tee to write all reads into checksum
	var checksum checksum
	teeReader := io.TeeReader(reader, &checksum)

	var header header
	err = binary.Read(teeReader, binary.LittleEndian, &header)
	if err != nil {
		return nil, 0, 0, err
	}
	if int(header.MessageID) >= len(messageFactory) {
		// Unknow message type, read into dummy until end of message
		dummy := make([]byte, header.PayloadLength+2)
		reader.Read(dummy)
		return nil, 0, 0, ErrUnknownMessageID(header.MessageID)
	}

	msg = messageFactory[header.MessageID]()
	if header.PayloadLength != msg.Size() {
		return nil, 0, 0, fmt.Errorf("Message ID and size don't match")
	}

	err = binary.Read(teeReader, binary.LittleEndian, msg)
	if err != nil {
		return nil, 0, 0, err
	}

	var streamChecksum uint16
	err = binary.Read(reader, binary.LittleEndian, &streamChecksum)
	if err != nil {
		return nil, 0, 0, err
	}
	if streamChecksum != checksum.sum {
		return nil, 0, 0, ErrInvalidChecksum(streamChecksum)
	}

	return msg, header.SystemID, header.ComponentID, nil
}
