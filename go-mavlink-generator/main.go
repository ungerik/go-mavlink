package main

import (
	// "encoding/xml"
	"github.com/ungerik/go-quick"
)

type MAVLink struct {
}

func main() {

	var format MAVLink

	err := quick.FileUnmarshallXML("https://raw.github.com/mavlink/mavlink/master/message_definitions/v1.0/common.xml", format)
	if err != nil {
		panic(err)
	}

}
