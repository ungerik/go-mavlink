package mavlink

type newMessageFunc func() Message

var messageFactory = []newMessageFunc{
	func() Message { return new(ExampleMessage) },
}

type Message interface {
	ID() uint8
	Size() uint8
}

type ExampleMessage struct {
	Test uint16
}

func (self *ExampleMessage) ID() uint8 {
	return 66
}

func (self *ExampleMessage) Size() uint8 {
	return 12
}
