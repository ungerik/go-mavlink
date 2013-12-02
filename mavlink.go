package mavlink

import (
	"encoding/binary"
	"io"
	// "io/ioutil"
)

var (
	frameStart      = []byte{'\xfe'}
	sequenceCounter uint8
)

type checksumWriter struct {
	writer io.Writer
	sum    uint16
}

func (self *checksumWriter) Write(data []byte) (n int, err error) {
	// todo calc sum

	return self.writer.Write(data)
}

func Send(writer io.Writer, systemID, componentID uint8, message Message) error {
	_, err := writer.Write(frameStart)
	if err != nil {
		return err
	}
	checksum := checksumWriter{writer: writer}

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

func Receive(reader io.Reader) (msg Message, systemID, componentID uint8, err error) {

	return nil, 0, 0, nil
}
