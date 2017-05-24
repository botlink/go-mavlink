package mavlink

import (
	"encoding/binary"
	"fmt"
)

// https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC%2020-165.pdf
// http://uavionix.com/downloads/integration/uAvionix%20Ping%20Integration%20Guide.pdf

// SDA Equipment Certification Level | Probability of Misleading Transmission
// Hazardous												 | <= 1 x 10^-7 per hour
// Major														 | <= 1 x 10^-5 per hour
// Minor														 | <= 1 x 10^-3 per hour
// Unknown													 | > 1 x 10^-3 per hour
//
// "Misleading" is defined as a transmission that contains false or inaccurate
// information to ATC or other interested party
//
// I'll give you one guess as to what you hand-built UAS pieced together from
// the lowest cost components you could find is certified to
const (
	// SDAUnknown indicates that the systems connected to the ADS-B is of an
	// unknown or unacceptable System Design Assurance level
	SDAUnknown = 0x00
	// SDAMinor indicates that the systems connected to the ADS-B are certified to
	// the SDA Minor level
	SDAMinor = 0x01
	// SDAMajor indicates that the systems connected to the ADS-B are certified to
	// the Major level
	SDAMajor = 0x02
	// SDAHazardous indicates that the systems connected to the ADS-B are certified to
	// the Hazardous level
	SDAHazardous = 0x03
)

// Source Integrity Level | Probability of Breaching Navigation Integrity
// Hazardous							| <= 1 x 10^-7 per hour
// Major									| <= 1 x 10^-5 per hour
// Minor									| <= 1 x 10^-3 per hour
// Unknown								| > 1 x 10^-3 per hour
//
// "Navigation Integrity" is the best-case navigation accuracy for the given
// source. This means that SIL is the chance that your broadcast position
// is less accuracy than its stated best case accuracy (for GPS, this is the
// chance that the circular error probability of your position is larger than
// normal).
const (
	// SILUnknown indicates that the navigation source has not been certified,
	// or has been certified to an unknown integrity level
	SILUnknown = 0x00
	// SILMinor indicates that the navigation source has been certified to the
	// Minor level
	SILMinor = 0x04
	// SILMajor indicates that the navigation source has been certified to the
	// Major level
	SILMajor = 0x08
	// SILHazardous indicates that the navigation source has been certified to the
	// Hazardous level
	SILHazardous = 0x0C
	// Who knows
	CSID = 0x10
	// ForceGNSSAltitude indicates to receivers that they should use the GNSS
	// derived altitude for positioning purposes
	ForceGNSSAltitude = 0x20
)

const (
	// MaxSpeedUnknown indicates that the maximum speed of the air vehicle is
	// unknown
	MaxSpeedUnknown = 0x00
	// MaxSpeedLessThan75Knots indicates that the maximum speed of the air vehicle
	// is greater than 0 KIAS but less than or equal to 75 KIAS
	MaxSpeedLessThan75Knots = 0x01
	// MaxSpeedLessThan150Knots indicates that the maximum speed of the air vehicle
	// is greater than 75 KIAS but less than or equal to 150 KIAS
	MaxSpeedLessThan150Knots = 0x02
	// MaxSpeedLessThan300Knots indicates that the maximum speed of the air vehicle
	// is greater than 150 KIAS less than or equal to 300 KIAS
	MaxSpeedLessThan300Knots = 0x03
	// MaxSpeedLessThan600Knots indicates that the maximum speed of the air vehicle
	// is greater than 300 KIAS but less than or equal to 600 KIAS
	MaxSpeedLessThan600Knots = 0x03
	// MaxSpeedLessThan1200Knots indicates that the maximum speed of the air vehicle
	// greater than 600 KIAS but less than or equal to 1200 KIAS
	MaxSpeedLessThan1200Knots = 0x04
	// MaxSpeedGreaterThan1200Knots indicates that the maximum speed of the air vehicle
	// is greater than 1200 KIAS
	MaxSpeedGreaterThan1200Knots = 0x06
	// ADSBInputNone indicates that the air vehicle cannot receive ADS-B messages
	ADSBInputNone = 0x00
	// ADSBInput1090 indicates that the air vehicle can receive ADS-B messages
	// on the 1090 MHz band
	ADSBInput1090 = 0x10
	// ADSBInput978 indicates that the air vehicle can receive ADS-B messages on
	// the 978 MHz band
	ADSBInput978 = 0x20
)

const (
	// AircraftSizeLengthWidthUnknown indicates that the air vehicle length and
	// width are not known
	AircraftSizeLengthWidthUnknown = 0x00
	// AircraftSizeLength15Width23 indicates that the aircraft has a fuselage less
	// Aircraft 15 meters or less long and a wingspan of less than 23 meters
	AircraftSizeLength15Width23 = 0x01
	// AircraftSizeLength25Width28p5 indicates that the aircraft has a fuselage less
	// than 15 meters long and a wingspan of less than 23 meters
	AircraftSizeLength25Width28p5 = 0x02
	// AircraftSizeLength25Width34 indicates that the aircraft has a fuselage less
	// than 25 meters long and a wingspan of less than 28.5 meters
	AircraftSizeLength25Width34 = 0x03
	// AircraftSizeLength35Width33 indicates that the aircraft has a fuselage less
	// than 35 meters long and a wingspan of less than 33 meters
	AircraftSizeLength35Width33 = 0x04
	// AircraftSizeLength35Width38 indicates that the aircraft has a fuselage less
	// than 35 meters long and a wingspan of less than 38 meters
	AircraftSizeLength35Width38 = 0x05
	// AircraftSizeLength45Width39p5 indicates that the aircraft has a fuselage less
	// than 45 meters long and a wingspan of less than 39.5 meters
	AircraftSizeLength45Width39p5 = 0x06
	// AircraftSizeLength45Width45 indicates that the aircraft has a fuselage less
	// than 45 meters long and a wingspan of less than 45 meters
	AircraftSizeLength45Width45 = 0x07
	// AircraftSizeLength55Width45 indicates that the aircraft has a fuselage less
	// than 55 meters long and a wingspan of less than 45 meters
	AircraftSizeLength55Width45 = 0x08
	// AircraftSizeLength55Width52 indicates that the aircraft has a fuselage less
	// than 55 meters long and a wingspan of less than 52 meters
	AircraftSizeLength55Width52 = 0x09
	// AircraftSizeLength65Width62p5 indicates that the aircraft has a fuselage less
	// than 65 meters long and a wingspan of less than 62.5 meters
	AircraftSizeLength65Width62p5 = 0x0A
	// AircraftSizeLength65Width67 indicates that the aircraft has a fuselage less
	// than 65 meters long and a wingspan of less than 67 meters
	AircraftSizeLength65Width67 = 0x0B
	// AircraftSizeLength75Width72p5 indicates that the aircraft has a fuselage less
	// than 75 meters long and a wingspan of less than 72.5 meters
	AircraftSizeLength75Width72p5 = 0x0C
	// AircraftSizeLength75Width80 indicates that the aircraft has a fuselage less
	// than 75 meters long and a wingspan of less than 80 meters
	AircraftSizeLength75Width80 = 0x0D
	// AircraftSizeLength85Width80 indicates that the aircraft has a fuselage less
	// than 85 meters long and a wingspan of less than 80 meters
	AircraftSizeLength85Width80 = 0x0E
	// AircraftSizeLength85Width90 indicates that the aircraft has a fuselage less
	// than 85 meters long and a wingspan of less than 90 meters
	AircraftSizeLength85Width90 = 0x0F
)

const (
	// GPSLateralOffsetUnknown indicates that the position of the navigation
	// source laterally offset from the centerline of the aircraft is unknown
	GPSLateralOffsetUnknown = 0x00
	// GPSLateralOffsetLeft2 indicates that the navigation source is offset left
	// from the aircraft centerline by 2 meters or less
	GPSLateralOffsetLeft2 = 0x01
	// GPSLateralOffsetLeft4 indicates that the navigation source is offset left
	// from the aircraft centerline by 4 meters or less
	GPSLateralOffsetLeft4 = 0x02
	// GPSLateralOffsetLeft6 indicates that the navigation source is offset left
	// from the aircraft centerline by 6 meters or less
	GPSLateralOffsetLeft6 = 0x03
	// GPSLateralOffsetRight0 indicates that the navigation source is offset right
	// from the aircraft centerline by 0 meters
	GPSLateralOffsetRight0 = 0x04
	// GPSLateralOffsetRight2 indicates that the navigation source is offset right
	// from the aircraft centerline by 2 meters or less
	GPSLateralOffsetRight2 = 0x05
	// GPSLateralOffsetRight4 indicates that the navigation source is offset right
	// from the aircraft centerline by 4 meters or less
	GPSLateralOffsetRight4 = 0x06
	// GPSLateralOffsetRight6 indicates that the navigation source is offset right
	// from the aircraft centerline by 6 meters or less
	GPSLateralOffsetRight6 = 0x07
)

const (
	// GPSLongitudinalOffsetUnkown indicates that the position of the navigation
	// source longitudinal offset from the aircraft nose is unknown
	GPSLongitudinalOffsetUnkown = 0x00
	// GPSLongitudinalOffsetAppliedBySensor indicates that the nose offset of the
	// navigation source is applied by the sensor
	GPSLongitudinalOffsetAppliedBySensor = 0x01
)

const (
	// UavionixStatusInitializing indicates the Ping module is initializing
	UavionixStatusInitializing = 0x00
	// UavionixStatusOK indicates the Ping module is operating normally
	UavionixStatusOK = 0x01
	// UavionixStatusTXFail1090 indicates the Ping module is unable to transmit
	// ADS-B messages on the 1090 MHz band due to a failure
	UavionixStatusTXFail1090 = 0x02
	// UavionixStatusRXFail1090 indicates the Ping module is unable to receive
	// ADS-B messages on the 1090 MHz band due to a failure
	UavionixStatusRXFail1090 = 0x04
	// UavionixStatusTXFailUAT indicates the Ping module is unable to transmit
	// ADS-B messages on the UAT band due to a failure
	UavionixStatusTXFailUAT = 0x08
	// UavionixStatusRXFailUAT indicates the Ping module is unable to receive
	// ADS-B messages on the UAT band due to a failure
	UavionixStatusRXFailUAT = 0x10
)

const (
	// AltitudeTypePressure indicates the altitude in the position report indicates
	// the altitude above mean sea level from the barometer
	AltitudeTypePressure = 0x00
	// AltitudeTypeGNSS indicates the altitude in position report indicates the
	// altitude above the WGS84 ellipsoid from the GNSS
	AltitudeTypeGNSS = 0x01
)

const (
	// EmitterTypeUnkown indicates that the sending vehicle doesn't understand
	// what it's supposed to be when it grows up
	EmitterTypeUnkown = 0x00
	// EmitterTypeLight indicates that the sending vehicle is a light aircraft
	EmitterTypeLight = 0x01
	// EmitterTypeSmall indicates that the sending vehicle is a small aircraft
	EmitterTypeSmall = 0x02
	// EmitterTypeLarge indicates that the sending vehicle is a large aircraft
	EmitterTypeLarge = 0x03
	// EmitterTypeLargeHighVortex indicates that the sending vehicle is a large
	// aircraft that creates a powerful vortex in flight
	EmitterTypeLargeHighVortex = 0x04
	// EmitterTypeHeavy indicates that the sending vehicle is a heavy aircraft
	EmitterTypeHeavy = 0x05
	// EmitterTypeHighlyManeuverable indicates that the sending vehicle is a
	// highly maneuverable aircraft
	EmitterTypeHighlyManeuverable = 0x06
	// EmitterTypeRotocraft indicates that the sending vehicle is a helicopter
	EmitterTypeRotocraft = 0x07
	// EmitterTypeUnassigned1 indicates that the sending vehicle is a type of
	// vehicle that has not been designated by the ICAO
	EmitterTypeUnassigned1 = 0x08
	// EmitterTypeGlider indicates that the sending vehicle is an unpowered glider
	EmitterTypeGlider = 0x09
	// EmitterTypeLighterThanAir indicates that the sending vehicle is a blimp or
	// zeppelin
	EmitterTypeLighterThanAir = 0x0A
	// EmitterTypeParachute indicates that the sending vehicle flies using
	// parachutes for lift
	EmitterTypeParachute = 0x0B
	// EmitterTypeUltralight indicates that the sending vehicle is an ultralight
	// aircraft
	EmitterTypeUltralight = 0x0C
	// EmitterTypeUnassigned2 indicates that the sending vehicle is a type of
	// vehicle that has not been designated by the ICAO
	EmitterTypeUnassigned2 = 0x0D
	// EmitterTypeUnmannedAerial indicates that the sending vehicle is an aircraft
	// with no pilot or crew on board
	EmitterTypeUnmannedAerial = 0x0E
	// EmitterTypeSpace indicates that the sending vehicle is a spacecraft
	EmitterTypeSpace = 0x0F
	// EmitterTypeUnassigned3 indicates that the sending vehicle is a type of
	// vehicle that has not been designated by the ICAO
	EmitterTypeUnassigned3 = 0x10
	// EmitterTypeSurfaceEmergency indicates that the sending vehicle is a ground
	// based emergency services vehicle
	EmitterTypeSurfaceEmergency = 0x11
	// EmitterTypeSurfaceService indicates that the sending vehicle is a ground
	// based normal service vehicle
	EmitterTypeSurfaceService = 0x12
	// EmitterTypePointObstacle indicates that the sending object is a ground
	// obstacle
	EmitterTypePointObstacle = 0x13
)

const (
	// ValidLatitudeLongitude indicates that the reported latitude and longitude
	// are valid
	ValidLatitudeLongitude = 0x0001
	// ValidAltitude indicates that the reported altitude is valid
	ValidAltitude = 0x0002
	// ValidHeading indicates that the reported heading is valid
	ValidHeading = 0x0004
	// ValidVelocity indicates that the reported horizontal velocity is valid
	ValidVelocity = 0x0008
	// ValidCallsign indicates that the reported callsign is valid
	ValidCallsign = 0x0010
	// ValidIdentity indicates that the reported identity is valid
	ValidIdentity = 0x0020
	// ValidSimulatedReport indicates that the message is from a simulation
	ValidSimulatedReport = 0x0040
	// ValidVerticalVelocity indicates that the reported vertical velocity is
	// valid
	ValidVerticalVelocity = 0x0080
	// ValidBarometer indicates that the reporting barometer is valid
	ValidBarometer = 0x0100
	// ValidSourceUAT indicates that the reporting ADS-B transponder is valid
	ValidSourceUAT = 0x8000
)

const (
	// StateIntentChange indicates that the aircraft state has changed or is
	// about to change
	StateIntentChange = 0x01
	// StateAutopilotEnabled indicates that the aircraft is being controlled
	// primarily by an autopilot system
	StateAutopilotEnabled = 0x02
	// StateBarometerCrosscheckEnabled indicates that the aircraft barometer and
	// GNSS are cross-checking altitudes
	StateBarometerCrosscheckEnabled = 0x04
	// StateOnGround indicates the aircraft is on the ground (not flying)
	StateOnGround = 0x08
	// StateIdentify indicates that the aircraft is identifying to ATC
	StateIdentify = 0x10
)

const (
	// ControlStandby indicates the aircraft identity systems are in standby mode
	ControlStandby = 0x00
	// ControlRXOnly indicates the aircraft can only receive, but not transmit
	// messages
	ControlRXOnly = 0x01
	// ControlTX1090Enable indicates the aircraft is broadcasting on ADS-B 1090
	// MHz bands
	ControlTX1090Enable = 0x02
	// ControlTXUATEnable indicates the aircraft is broadcasting on ADS-B UAT
	// bands
	ControlTXUATEnable = 0x04
	// ControlModeAEnabled indicates the aircraft is transmitting using its Mode A
	// transponder
	ControlModeAEnabled = 0x08
	// ControlModeCEnabled indicates the aircraft is transmitting using its Mode C
	// transponder
	ControlModeCEnabled = 0x10
	// ControlModeSEnabled indicates the aircraft is transmitting using its Mode S
	// transponder
	ControlModeSEnabled = 0x20
)

const (
	// FixTypeGPSNone1 indicates that the aircraft does not have a GPS fix
	FixTypeGPSNone1 = 0x00
	// FixTypeGPSNone2 indicates that the aircraft does not have a GPS fix
	FixTypeGPSNone2 = 0x01
	// FixTypeGPS2D indicates that the aircraft has a 2D (horizontal) GPS fix
	FixTypeGPS2D = 0x02
	// FixTypeGPS3D indicates that the aircraft has a 3D GPS fix
	FixTypeGPS3D = 0x03
	// FixTypeGPSDifferential indicates that aircraft has a differential GPS fix
	FixTypeGPSDifferential = 0x04
	// FixTypeGPSRTK indicates that the aircraft has an RTK GPS fix
	FixTypeGPSRTK = 0x05
)

// OwnShipStatic represents aircraft data that changes infrequently
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

// MsgID returns the message ID of the OwnShipStatic message
func (self *OwnShipStatic) MsgID() uint8 {
	return 201
}

// MsgName returns the message name of the OwnShipStatic message
func (self *OwnShipStatic) MsgName() string {
	return "OwnShipStatic"
}

// Unpack parses the contents of the packet into the fields of the OwnShipStatic
// message
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

// Pack returns a byte-array representation of the the OwnShipStatic message
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

// OwnShipDynamic represents aircraft data that changes frequently
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

// MsgID returns the message ID of the OwnShipDynamic message
func (self *OwnShipDynamic) MsgID() uint8 {
	return 202
}

// MsgName returns the message name of the OwnShipDynamic message
func (self *OwnShipDynamic) MsgName() string {
	return "OwnShipDynamic"
}

// Unpack parses the contents of the packet into the fields of the OwnShipDynamic
// message
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

// Pack returns a byte-array representation of the OwnShipDynamic message
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

// UavionixStatus represents the status of the Uavionix transponder
type UavionixStatus struct {
	Status uint8
}

// MsgID returns the message ID of the UavionixStatus message
func (self *UavionixStatus) MsgID() uint8 {
	return 203
}

// MsgName returns the message name of the UavionixStatus message
func (self *UavionixStatus) MsgName() string {
	return "Status"
}

// Unpack parses the contents of the packet into the fields of the UavionixStatus
// message
func (self *UavionixStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 1 {
		return fmt.Errorf("payload too small")
	}

	self.Status = uint8(p.Payload[0])

	return nil
}

// Pack returns a byte-array representation of the UavionixStatus message
func (self *UavionixStatus) Pack(p *Packet) error {
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

// MsgID returns the message ID of the TrafficReport message
func (self *TrafficReport) MsgID() uint8 {
	return 246
}

// MsgName returns the message name of the TrafficReport message
func (self *TrafficReport) MsgName() string {
	return "TrafficReport"
}

// Unpack parses the packet contents into the fields of the TrafficReport message
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

// Pack returns a byte-array representation of the TrafficReport message
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
