package mavlink

import (
	"encoding/binary"
	"fmt"
)

const (
	SDA_UNKNOWN   = 0
	SDA_MINOR     = 1
	SDA_MAJOR     = 2
	SDA_HAZARDOUS = 3
)

type OwnShipStatic struct {
	ICAOAddress           [3]uint8
	Integrity             uint8
	StallSpeed            uint16
	Callsign              [8]byte
	Capability            uint8
	Emitter               uint8
	ALWEncoded            uint8
	GPSLateralOffset      uint8
	GPSLongitudinalOffset uint8
}

func (self *OwnShipStatic) MsgID() uint8 {
	return 201
}

func (self *OwnShipStatic) MsgName() string {
	return "OwnShipStatic"
}

func (self *OwnShipStatic) Unpack(p *Packet) error {
	if len(p.Payload) < 19 {
		return fmt.Errorf("payload too small")
	}

	copy(self.ICAOAddress[:], p.Payload[0:2])

	self.Integrity = uint8(p.Payload[3])
	self.StallSpeed = binary.LittleEndian.Uint16(p.Payload[4:])

	copy(self.Callsign[:], p.Payload[6:13])

	self.Capability = uint8(p.Payload[14])
	self.Emitter = uint8(p.Payload[15])
	self.ALWEncoded = uint8(p.Payload[16])
	self.GPSLateralOffset = uint8(p.Payload[17])
	self.GPSLongitudinalOffset = uint8(p.Payload[18])

	return nil
}

func (self *OwnShipStatic) Pack(p *Packet) error {
	payload := make([]byte, 19)

	copy(payload[0:], self.ICAOAddress[:])

	payload[3] = byte(self.Integrity)
	binary.LittleEndian.PutUint16(payload[4:], self.StallSpeed)

	copy(payload[6:], self.Callsign[:])

	payload[14] = byte(self.Capability)
	payload[15] = byte(self.Emitter)
	payload[16] = byte(self.ALWEncoded)
	payload[17] = byte(self.GPSLateralOffset)
	payload[18] = byte(self.GPSLongitudinalOffset)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

type OwnShipDynamic struct {
	GPSTime            uint32
	Latitude           int32
	Longitude          int32
	PressureAltitude   int32
	GPSAltitude        int32
	GPSHorizontalFOM   uint32
	GPSVerticalFOM     uint16
	VelocityAccuracy   uint16
	VerticalVelocity   int16
	NorthSouthVelocity int16
	EastWestVelocity   int16
	State              uint16
	Squawk             uint16
	FixType            uint8
	GPSSatellites      uint8
	EmergencyStatus    uint8
	Control            uint8
}

func (self *OwnShipDynamic) MsgID() uint8 {
	return 202
}

func (self *OwnShipDynamic) MsgName() string {
	return "OwnShipDynamic"
}

func (self *OwnShipDynamic) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}

	self.GPSTime = binary.LittleEndian.Uint32(p.Payload[0:])
	self.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.PressureAltitude = int32(int32(binary.LittleEndian.Uint32(p.Payload[12:])))
	self.GPSAltitude = int32(int32(binary.LittleEndian.Uint32(p.Payload[16:])))
	self.GPSHorizontalFOM = binary.LittleEndian.Uint32(p.Payload[20:])
	self.GPSVerticalFOM = binary.LittleEndian.Uint16(p.Payload[24:])
	self.VelocityAccuracy = binary.LittleEndian.Uint16(p.Payload[26:])
	self.VerticalVelocity = int16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.NorthSouthVelocity = int16(binary.LittleEndian.Uint16(p.Payload[30:]))
	self.EastWestVelocity = int16(binary.LittleEndian.Uint16(p.Payload[32:]))
	self.State = binary.LittleEndian.Uint16(p.Payload[34:])
	self.Squawk = binary.LittleEndian.Uint16(p.Payload[36:])

	self.FixType = uint8(p.Payload[38])
	self.GPSSatellites = uint8(p.Payload[39])
	self.EmergencyStatus = uint8(p.Payload[40])
	self.Control = uint8(p.Payload[41])

	return nil
}

func (self *OwnShipDynamic) Pack(p *Packet) error {
	payload := make([]byte, 42)

	binary.LittleEndian.PutUint32(payload[0:], self.GPSTime)
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Latitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Longitude))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.PressureAltitude))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.GPSAltitude))
	binary.LittleEndian.PutUint32(payload[20:], self.GPSHorizontalFOM)
	binary.LittleEndian.PutUint16(payload[24:], self.GPSVerticalFOM)
	binary.LittleEndian.PutUint16(payload[26:], self.VelocityAccuracy)
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.VerticalVelocity))
	binary.LittleEndian.PutUint16(payload[30:], uint16(self.NorthSouthVelocity))
	binary.LittleEndian.PutUint16(payload[32:], uint16(self.EastWestVelocity))
	binary.LittleEndian.PutUint16(payload[34:], self.State)
	binary.LittleEndian.PutUint16(payload[36:], self.Squawk)
	payload[38] = byte(self.FixType)
	payload[39] = byte(self.GPSSatellites)
	payload[40] = byte(self.EmergencyStatus)
	payload[41] = byte(self.Control)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

type Status struct {
	Status uint8
}

func (self *Status) MsgID() uint8 {
	return 203
}

func (self *Status) MsgName() string {
	return "Status"
}

func (self *Status) Unpack(p *Packet) error {
	if len(p.Payload) < 1 {
		return fmt.Errorf("payload too small")
	}

	self.Status = uint8(p.Payload[0])

	return nil
}

func (self *Status) Pack(p *Packet) error {
	payload := make([]byte, 1)

	payload[0] = byte(self.Status)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

// TrafficReport represents an ADS-B traffic report for an individual vehicle
type TrafficReport struct {
	ICAOAddress                uint32
	Latitude                   int32
	Longitude                  int32
	Altitude                   int32
	Heading                    uint16
	HorizontalVelocity         uint16
	VerticalVelocity           int16
	ValidFlags                 uint16
	Squawk                     uint16
	AltitudeType               uint8
	Callsign                   [9]byte
	EmitterType                uint8
	TimeSinceLastCommunication uint8
}

func (self *TrafficReport) MsgID() uint8 {
	return 246
}

func (self *TrafficReport) MsgName() string {
	return "TrafficReport"
}

func (self *TrafficReport) Unpack(p *Packet) error {
	if len(p.Payload) < 38 {
		return fmt.Errorf("payload too small")
	}

	self.ICAOAddress = binary.LittleEndian.Uint32(p.Payload[0:])
	self.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Heading = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.HorizontalVelocity = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.VerticalVelocity = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.ValidFlags = binary.LittleEndian.Uint16(p.Payload[22:])
	self.Squawk = binary.LittleEndian.Uint16(p.Payload[24:])
	self.AltitudeType = uint8(p.Payload[26])

	copy(self.Callsign[:], p.Payload[27:35])

	self.EmitterType = uint8(p.Payload[36])
	self.TimeSinceLastCommunication = uint8(p.Payload[37])

	return nil
}

func (self *TrafficReport) Pack(p *Packet) error {
	payload := make([]byte, 38)
	binary.LittleEndian.PutUint32(payload[0:], self.ICAOAddress)
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Latitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Longitude))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Altitude))
	binary.LittleEndian.PutUint16(payload[16:], self.Heading)
	binary.LittleEndian.PutUint16(payload[18:], self.HorizontalVelocity)
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.VerticalVelocity))
	binary.LittleEndian.PutUint16(payload[22:], self.ValidFlags)
	binary.LittleEndian.PutUint16(payload[24:], self.Squawk)
	payload[26] = byte(self.AltitudeType)
	copy(payload[27:], self.Callsign[:])
	payload[36] = byte(self.EmitterType)
	payload[37] = byte(self.TimeSinceLastCommunication)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

// DialectUAVonix is the MAVLink dialect containing messages used by the
// UAVonix ping2020 ADS-B system
var DialectUAVonix *Dialect = &Dialect{
	Name: "uavonix",
	crcExtras: map[uint8]uint8{
		246: 184,
		203: 85,
		202: 7,
		201: 126,
	},
}
