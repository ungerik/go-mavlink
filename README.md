#go-mavlink


go impletation of the MAVLink protocol.
This implementation is mainly inspired by the C version (see [on GitHub](https://github.com/mavlink/qgroundcontrol/tree/master/libs/mavlink/include/mavlink/v1.0)).
Only tested under Unix


#Usage

Here two quick examples to understand how to deal with it but first, look at the MavPacket definition.

###MavPacket definition :
```go
type Message interface {
        ID() uint8
        Size() uint8
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

```

You got all the message definitions in message.go.


##UDP communication (receiving example)


```go
package main

import (
	"bufio"
	"fmt"
	mav "github.com/Sabmit/go-mavlink"
	"log"
	"net"
)

func main() {
	laddr, _ := net.ResolveUDPAddr("udp", "127.0.0.1:14550")
	conn, _ := net.ListenUDP("udp", laddr) // check errors

	parser := mav.GetMavParser()
    reader := bufio.NewReader(conn)

    for {
    	c, _ := reader.ReadByte() // check errors

        packet, err := parser(c)
        if err != nil {
        	log.Fatalf("Parser error: ", err)
        } else if packet != nil {
        	fmt.Println("Packet received :", packet.Bytes())
    	}
	}
}
```
The C server is available [here](https://github.com/mavlink/mavlink/tree/master/examples/linux), you can also test it by hand...
```bash
$ echo -ne "\xfe\x09\x0\x01\xC8\x00\x0\x0\x0\x0\x0\x0\x0\x0\x0\x5A\x3E" | nc -u 127.0.0.1 14550
```



##Serial communication (Sending example)
```go
import (
	"github.com/tarm/goserial"
	mav "github.com/Sabmit/go-mavlink"
       "log"
)


func main() {
	port := &serial.Config{Name: "/dev/ttyUSB0", Baud: 57600}
        s, _ := serial.OpenPort(port) // check errors

	message := new(mav.RequestDataStream)
        message.ReqMessageRate = 20

	err := mav.Send(s, 1, 200, message)
        if err != nil {
    	log.Fatalf("Error while sending the packet:", err)
    }
}
```

##More
The operation is simple : get the MavParser and then, parse each received byte untill you get a non-nil pointer pointing to the full MavPacket.

Please read `main_udp.go` and `main_serial.go` if you want the full code examples.



#TO-DO
* For now, only the received packet has its own checksum computed on the go
* Verify if all properties of each message is in its own place (see [Mavlink generator](http://www.qgroundcontrol.org/mavlink/generator))


#See
* [Mavlink documentation](https://pixhawk.ethz.ch/mavlink/)
* [QGroundControl](http://www.qgroundcontrol.org/mavlink/start)


#Authors

[ungerik]: https://github.com/ungerik
[sabmit]: http://github.com/sabmit
