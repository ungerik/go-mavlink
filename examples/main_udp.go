package main

import (
	"time"
	"fmt"
	"net"
	"log"
	"io"
	"bufio"
	mav "go-mavlink"
//	mav "github.com/Sabmit/go-mavlink"
)

func readPacketJob(r io.Reader, rxChan chan <- *mav.MavPacket) {
	parser := mav.GetMavParser()
	reader := bufio.NewReader(r)

	for {
		c, err := reader.ReadByte()
		if err != nil {
			log.Fatalf("Could not read: ", err)
		}
		packet, err := parser(c)
		if err != nil {
			log.Fatalf("Parser error: ", err)
		} else if packet != nil {
			rxChan <- packet
		}
	}
}

func sendPacketJob(c *net.UDPConn, txChan chan <- *mav.MavPacket) {
	raddr, err := net.ResolveUDPAddr("udp", "127.0.0.1:14551")
	if err != nil {
		log.Fatalf("Error can not resolve UDP address:", err)
	}
	for {
		message := new(mav.Heartbeat)
		packet, err := mav.CreatePacket(1, 200, message)
		if err != nil {
			log.Fatalf("Error with the packet's creation:", err)
		}

		_, err = c.WriteTo(packet.Bytes(), raddr)
		if err != nil {
			log.Fatalf("Cannot write packet:", err)
		}
		txChan <- packet
		time.Sleep(1000 * time.Millisecond)
	}
}

func getConnection() *net.UDPConn {
	laddr, err := net.ResolveUDPAddr("udp", "127.0.0.1:14550")
	if err != nil {
		log.Fatalf("Could not resolve UDP address: ", err)
	}

	conn, err := net.ListenUDP("udp", laddr)
	if err != nil {
		log.Fatalf("Could not listen: ", err)
	}

	return conn
}

func main() {
	var packet *mav.MavPacket
	var cRx chan *mav.MavPacket = make(chan *mav.MavPacket, 100)
	var cTx chan *mav.MavPacket = make(chan *mav.MavPacket)

	conn := getConnection()
	defer conn.Close()

	go readPacketJob(conn, cRx)
	go sendPacketJob(conn, cTx)

	for {
		select {
		case packet = <- cRx:
			fmt.Println("Packet received :", packet.Bytes())
		case packet = <- cTx:
			fmt.Println("Packet sent :", packet.Bytes())
		}
	}
}
