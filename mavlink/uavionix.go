package mavlink

import (
	"encoding/binary"
	"fmt"
)

//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

// UavionixSdaLevel: System Design Assurance Level. Indicates the probability per hour of a misleading ADS-B transmission
const (
	UAVIONIX_SDA_LEVEL_UNKNOWN   = 0 // Probability greater than 1x10^-3 per hour
	UAVIONIX_SDA_LEVEL_MINOR     = 1 // Probability less than or equal to 1x10^-3 per hour
	UAVIONIX_SDA_LEVEL_MAJOR     = 2 // Probability less than or equal to 1x10^-5 per hour
	UAVIONIX_SDA_LEVEL_HAZARDOUS = 3 // Probability less than or equal to 1x10^-7 per hour
)

// UavionixSilLevel: Source Integrity Level. Indicates the probability that the aircraft position accuracy is less than normal.
const (
	UAVIONIX_SIL_LEVEL_UNKNOWN             = 0  // Probability greater than 1x10^-3 per hour
	UAVIONIX_SIL_LEVEL_MINOR               = 4  // Probability less than or equal to 1x10^-3 per hour
	UAVIONIX_SIL_LEVEL_MAJOR               = 8  // Probability less than or equal to 1x10^-5 per hour
	UAVIONIX_SIL_LEVEL_HAZARDOUS           = 12 // Probability less than or equal to 1x10^-7 per hour
	UAVIONIX_SIL_LEVEL_CSID                = 16 // Unknown
	UAVIONIX_SIL_LEVEL_FORCE_GNSS_ALTITUDE = 32 // Force use of GNSS altitude
)

// UavionixVehicleCapability: Performace characteristics of the vehicle.
const (
	UAVIONIX_VEHICLE_CAPABILITY_MAX_SPEED_UNKNOWN  = 0  // Max speed of the vehicle is unknown
	UAVIONIX_VEHICLE_CAPABILITY_MAX_SPEED_LT75     = 1  // Max speed of the vehicle is less than 75 knots
	UAVIONIX_VEHICLE_CAPABILITY_MAX_SPEED_LT150    = 2  // Max speed of the vehicle is less than 150 knots
	UAVIONIX_VEHICLE_CAPABILITY_MAX_SPEED_LT300    = 3  // Max speed of the vehicle is less than 300 knots
	UAVIONIX_VEHICLE_CAPABILITY_MAX_SPEED_LT600    = 4  // Max speed of the vehicle is less than 600 knots
	UAVIONIX_VEHICLE_CAPABILITY_MAX_SPEED_LT1200   = 5  // Max speed of the vehicle is less than 1200 knots
	UAVIONIX_VEHICLE_CAPABILITY_MAX_SPEED_GT1200   = 6  // Max speed of the vehicle is greater than 1200 knots
	UAVIONIX_VEHICLE_CAPABILITY_ADSB_INPUT_1090MHZ = 16 // Indicates that the air vehicle can receive ADS-B messages on 1090MHZ bands
	UAVIONIX_VEHICLE_CAPABILITY_ADSB_INPUT_UAT     = 32 // Indicates that the air vehicle can receive ADS-B messages on UAT bands
)

// UavionixAircraftSize: Size of the aircraft.
const (
	UAVIONIX_AIRCRAFT_SIZE_UNKNOWN   = 0  // Aircraft length and wingspan are unknown
	UAVIONIX_AIRCRAFT_SIZE_L15_W23   = 1  // Aircraft is 15 meters or less long and a wingspan of less than 23 meters
	UAVIONIX_AIRCRAFT_SIZE_L25_W28P5 = 2  // Aircraft is 25 meters or less long and a wingspan of less than 28.5 meters
	UAVIONIX_AIRCRAFT_SIZE_L25_W34   = 3  // Aircraft is 25 meters or less long and a wingspan of less than 34 meters
	UAVIONIX_AIRCRAFT_SIZE_L35_W33   = 4  // Aircraft is 35 meters or less long and a wingspan of less than 33 meters
	UAVIONIX_AIRCRAFT_SIZE_L35_W38   = 5  // Aircraft is 35 meters or less long and a wingspan of less than 38 meters
	UAVIONIX_AIRCRAFT_SIZE_L45_W39P5 = 6  // Aircraft is 45 meters or less long and a wingspan of less than 39.5 meters
	UAVIONIX_AIRCRAFT_SIZE_L45_W45   = 7  // Aircraft is 45 meters or less long and a wingspan of less than 45 meters
	UAVIONIX_AIRCRAFT_SIZE_L55_W45   = 8  // Aircraft is 55 meters or less long and a wingspan of less than 45 meters
	UAVIONIX_AIRCRAFT_SIZE_L55_W52   = 9  // Aircraft is 55 meters or less long and a wingspan of less than 52 meters
	UAVIONIX_AIRCRAFT_SIZE_L65_W62P5 = 10 // Aircraft is 65 meters or less long and a wingspan of less than 62.5 meters
	UAVIONIX_AIRCRAFT_SIZE_L65_W67   = 11 // Aircraft is 65 meters or less long and a wingspan of less than 67 meters
	UAVIONIX_AIRCRAFT_SIZE_L75_W72P5 = 12 // Aircraft is 75 meters or less long and a wingspan of less than 72.5 meters
	UAVIONIX_AIRCRAFT_SIZE_L75_W80   = 13 // Aircraft is 75 meters or less long and a wingspan of less than 80 meters
	UAVIONIX_AIRCRAFT_SIZE_L85_W80   = 14 // Aircraft is 85 meters or less long and a wingspan of less than 80 meters
	UAVIONIX_AIRCRAFT_SIZE_L85_W90   = 15 // Aircraft is 85 meters or less long and a wingspan of less than 90 meters
)

// UavionixGpsOffset: Indicates the offset of the GPS from nose and centerline.
const (
	UAVIONIX_GPS_OFFSET_LATERAL_UNKNOWN                = 0 // Unknown lateral offset
	UAVIONIX_GPS_OFFSET_LATERAL_LEFT_2                 = 1 // Lateral offset left 2 meters
	UAVIONIX_GPS_OFFSET_LATERAL_LEFT_4                 = 2 // Lateral offset left 4 meters
	UAVIONIX_GPS_OFFSET_LATERAL_LEFT_6                 = 3 // Lateral offset left 6 meters
	UAVIONIX_GPS_OFFSET_LATERAL_RIGHT_0                = 4 // Lateral offset right 0 meters
	UAVIONIX_GPS_OFFSET_LATERAL_RIGHT_2                = 5 // Lateral offset right 2 meters
	UAVIONIX_GPS_OFFSET_LATERAL_RIGHT_4                = 6 // Lateral offset right 4 meters
	UAVIONIX_GPS_OFFSET_LATERAL_RIGHT_6                = 7 // Lateral offset right 6 meters
	UAVIONIX_GPS_OFFSET_LONGITUDINAL_UNKNOWN           = 8 // Longitudinal offset unknown
	UAVIONIX_GPS_OFFSET_LONGITUDINAL_SENSOR_CALIBRATED = 1 // Longitudinal offset applied by sensor
)

// UavionixStatus: Uavionix ADS-B system status
const (
	UAVIONIX_STATUS_INITIALIZING = 0  // The module is initializing
	UAVIONIX_STATUS_OK           = 1  // The module is operating normally
	UAVIONIX_STATUS_TX_FAIL_1090 = 2  // The module can't transmit on 1090 MHZ
	UAVIONIX_STATUS_RX_FAIL_1090 = 4  // The module can't receive on 1090 MHZ
	UAVIONIX_STATUS_TX_FAIL_UAT  = 8  // The module can't transmit on UAT bands
	UAVIONIX_STATUS_RX_FAIL_UAT  = 16 // The module can't receive on UAT bands
)

// UavionixAltitudeType: Altitude source for vehicle
const (
	UAVIONIX_ALTITUDE_PRESSURE = 0 // Altitude above mean sea level from the barometer
	UAVIONIX_ALTITUDE_GNSS     = 1 // Altitude above the WGS84 ellipsoid from the GNSS
)

// UavionixEmitterAircraftType: ADS-B emitter vehicle type
const (
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_UNKNOWN             = 0  // Emitter vehicle type unknown
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_LIGHT               = 1  // Emitter vehicle type light
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_SMALL               = 2  // Emitter vehicle type small
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_LARGE               = 3  // Emitter vehicle type large
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_LARGE_HIGH_VORTEX   = 4  // Emitter vehicle type large with high vortex
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_HEAVY               = 5  // Emitter vehicle type heavy
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_HIGHLY_MANEUVERABLE = 6  // Emitter vehicle type highly maneuverable
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_ROTOCRAFT           = 7  // Emitter vehicle type rotocraft
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_UNASSIGNED1         = 8  // Emitter vehicle type unassigned
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_GLIDER              = 9  // Emitter vehicle type glider
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_LIGHTER_THAN_AIR    = 10 // Emitter vehicle type lighter than air
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_PARACHUTE           = 11 // Emitter vehicle type parachute
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_ULTRALIGHT          = 12 // Emitter vehicle type ultralight
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_UNASSIGNED2         = 13 // Emitter vehicle type unassigned
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_UAV                 = 14 // Emitter vehicle type UAV
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_SPACE               = 15 // Emitter vehicle type space
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_UNASSIGNED3         = 16 // Emitter vehicle type unassigned
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_SURFACE_EMERGENCY   = 17 // Emitter vehicle type surface emergency
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_SURFACE_SERVICE     = 18 // Emitter vehicle type surface service
	UAVIONIX_EMITTER_AIRCRAFT_TYPE_POINT_OBSTACLE      = 19 // Emitter vehicle type point obstacle
)

// UavionixValidFlag: Valid bits for data transmitted or received
const (
	UAVIONIX_VALID_FLAG_LATLON            = 1   // Latitude and longitude are valid
	UAVIONIX_VALID_FLAG_ALT               = 2   // Altitude valid
	UAVIONIX_VALID_FLAG_HEADING           = 4   // Heading valid
	UAVIONIX_VALID_FLAG_VELOCITY          = 8   // Velocity valid
	UAVIONIX_VALID_FLAG_CALLSIGN          = 16  // Callsign valid
	UAVIONIX_VALID_FLAG_IDENT             = 32  // Ident valid
	UAVIONIX_VALID_FLAG_SIMULATED         = 64  // Simulated data
	UAVIONIX_VALID_FLAG_VERTICAL_VELOCITY = 128 // Vertical velocity valid
	UAVIONIX_VALID_FLAG_BAROMETER         = 8   // Barometer valid
	UAVIONIX_VALID_FLAG_UAT               = 9   // UAT transponder valid
)

// UavionixAircraftState: State of the aircraft
const (
	UAVIONIX_AIRCRAFT_STATE_INTENT_CHANGE                = 1  // Intent to change course/altitude/speed
	UAVIONIX_AIRCRAFT_STATE_AP_ENABLED                   = 2  // Autopilot enabled
	UAVIONIX_AIRCRAFT_STATE_BAROMETER_CROSSCHECK_ENABLED = 4  // Altitude source cross check
	UAVIONIX_AIRCRAFT_STATE_ON_GROUND                    = 8  // On the ground
	UAVIONIX_AIRCRAFT_STATE_IDENT                        = 16 // Ident for ATC
)

// UavionixAircraftControl: ATC communication and control systems
const (
	UAVIONIX_AIRCRAFT_CONTROL_STANDBY        = 0  // Systems on standby
	UAVIONIX_AIRCRAFT_CONTROL_RX_ONLY        = 1  // Systems only able to receive, not transmit
	UAVIONIX_AIRCRAFT_CONTROL_TX1090_ENABLED = 2  // Systems broadcasting on 1090MHZ
	UAVIONIX_AIRCRAFT_CONTROL_TXUAT_ENABLE   = 4  // Systems broadcasting on UAT bands
	UAVIONIX_AIRCRAFT_CONTROL_MODEA_ENABLE   = 8  // Systems broadcasting Mode A identifier
	UAVIONIX_AIRCRAFT_CONTROL_MODEC_ENABLE   = 16 // Systems broadcasting Mode C identifier
	UAVIONIX_AIRCRAFT_CONTROL_MODES_ENABLE   = 32 // Systems broadcasting Mode S identifier
)

// UavionixGpsFixType: GPS fix type
const (
	UAVIONIX_GPS_FIX_TYPE_NONE1   = 0 // No fix
	UAVIONIX_GPS_FIX_TYPE_NONE2   = 1 // No fix
	UAVIONIX_GPS_FIX_TYPE_GPS2D   = 2 // 2D Fix
	UAVIONIX_GPS_FIX_TYPE_GPS3D   = 3 // 3D Fix
	UAVIONIX_GPS_FIX_TYPE_GPSDIFF = 4 // Differential GPS system fix
	UAVIONIX_GPS_FIX_TYPE_GPSRTK  = 5 // RTK GPS system fix
)

// Describes other air traffic around the UAV
type TrafficReport struct {
	IcaoAddress  uint32  // ICAO Address
	Lat          int32   // The reported latitude in degrees * 1E7
	Lon          int32   // The reported longitude in degrees * 1E7
	Altitude     int32   // Altitude in Meters * 1E3 (up is +ve) - Check ALT_TYPE for reference datum
	Heading      uint16  // Course over ground in degrees * 10^2
	HorVelocity  uint16  // The horizontal velocity in (m/s * 1E2)
	VerVelocity  int16   // The vertical velocity in (m/s * 1E2)
	Validflags   uint16  // Valid data fields
	Squawk       uint16  // Mode A Squawk code (0xFFFF = no code)
	AltitudeType uint8   // Altitude Type
	Callsign     [9]byte // The callsign
	EmitterType  uint8   // Emitter Category
	Tslc         uint8   // Time since last communication in seconds
}

func (self *TrafficReport) MsgID() uint8 {
	return 246
}

func (self *TrafficReport) MsgName() string {
	return "TrafficReport"
}

func (self *TrafficReport) Pack(p *Packet) error {
	payload := make([]byte, 38)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.IcaoAddress))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Altitude))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Heading))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.HorVelocity))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.VerVelocity))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.Validflags))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Squawk))
	payload[26] = byte(self.AltitudeType)
	copy(payload[27:], self.Callsign[:])
	payload[36] = byte(self.EmitterType)
	payload[37] = byte(self.Tslc)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *TrafficReport) Unpack(p *Packet) error {
	if len(p.Payload) < 38 {
		return fmt.Errorf("payload too small")
	}
	self.IcaoAddress = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Heading = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.HorVelocity = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.VerVelocity = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.Validflags = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.Squawk = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.AltitudeType = uint8(p.Payload[26])
	copy(self.Callsign[:], p.Payload[27:36])
	self.EmitterType = uint8(p.Payload[36])
	self.Tslc = uint8(p.Payload[37])
	return nil
}

// Describes the status of the Uavionix ADS-B transponder
type UavionixStatus struct {
	Status uint8 // Self test status
}

func (self *UavionixStatus) MsgID() uint8 {
	return 203
}

func (self *UavionixStatus) MsgName() string {
	return "UavionixStatus"
}

func (self *UavionixStatus) Pack(p *Packet) error {
	payload := make([]byte, 1)
	payload[0] = byte(self.Status)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *UavionixStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 1 {
		return fmt.Errorf("payload too small")
	}
	self.Status = uint8(p.Payload[0])
	return nil
}

// Describes aircraft data that changes frequently
type OwnShipDynamic struct {
	Utctime  uint32 // UTC time in since GPS epoch (in s since Jan 6, 1980). If unknown set to UINT32_MAX
	Lat      int32  // The reported latitude in degrees * 1E7.  If unknown set to INT32_MAX
	Lon      int32  // The reported longitude in degrees * 1E7.  If unknown set to INT32_MAX
	Altpres  int32  // Barometric pressure altitude relative to a standard atmosphere of 1013.2 mBar and NOT bar corrected altitude (meters * 1E3) UP +ve. If unknown set to INT32_MAX
	Altgnss  int32  // Altitude (meters * 1E3). (up +ve). WGS84 altitude. If unknown set to INT32_MAX
	Acchoriz uint32 // Horizontal accuracy(HFOM) (mm). If unknown set to UINT32_MAX
	Accvert  uint16 // Vertical accuracy(VFOM) (cm). If unknown set to UINT16_MAX
	Accvel   uint16 // Velocity accuracy (m/s * 1E3). If unknown set to UINT16_MAX.
	Velvert  int16  // GPS vertical speed (m/s * 1E2). If unknown set to INT16_MAX.
	Nsvog    int16  // North-South velocity over ground (m/s * 1E2) North +ve. If unknown set to INT16_MAX.
	Ewvog    int16  // East-West velocity over ground (m/s * 1E2) East +ve. If unknown set to INT16_MAX.
	State    uint16 // ADS-B input flags
	Squawk   uint16 // Mode A code (typically 1200 [0x04B0] for VFR)
	Fixtype  uint8  // GPS Fix.
	Numsats  uint8  // Number of satellites visible. If unknown set to UINT8_MAX.
	Emstatus uint8  // Emergency status (table 2-78 of DO-260B).
	Control  uint8  // ADS-B transponder dynamic input control flags
}

func (self *OwnShipDynamic) MsgID() uint8 {
	return 202
}

func (self *OwnShipDynamic) MsgName() string {
	return "OwnShipDynamic"
}

func (self *OwnShipDynamic) Pack(p *Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Utctime))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Altpres))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Altgnss))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Acchoriz))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Accvert))
	binary.LittleEndian.PutUint16(payload[26:], uint16(self.Accvel))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.Velvert))
	binary.LittleEndian.PutUint16(payload[30:], uint16(self.Nsvog))
	binary.LittleEndian.PutUint16(payload[32:], uint16(self.Ewvog))
	binary.LittleEndian.PutUint16(payload[34:], uint16(self.State))
	binary.LittleEndian.PutUint16(payload[36:], uint16(self.Squawk))
	payload[38] = byte(self.Fixtype)
	payload[39] = byte(self.Numsats)
	payload[40] = byte(self.Emstatus)
	payload[41] = byte(self.Control)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *OwnShipDynamic) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	self.Utctime = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Altpres = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Altgnss = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Acchoriz = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Accvert = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.Accvel = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	self.Velvert = int16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.Nsvog = int16(binary.LittleEndian.Uint16(p.Payload[30:]))
	self.Ewvog = int16(binary.LittleEndian.Uint16(p.Payload[32:]))
	self.State = uint16(binary.LittleEndian.Uint16(p.Payload[34:]))
	self.Squawk = uint16(binary.LittleEndian.Uint16(p.Payload[36:]))
	self.Fixtype = uint8(p.Payload[38])
	self.Numsats = uint8(p.Payload[39])
	self.Emstatus = uint8(p.Payload[40])
	self.Control = uint8(p.Payload[41])
	return nil
}

// Describes aircraft data that changes less frequently
type OwnShipStatic struct {
	Stallspeed uint16   // Aircraft stall speed in cm/s.
	Icao       [3]uint8 // Vehicle address (24 bits). Byte[2] = msByte
	Integrity  uint8    // System Integrity and Design Assurance
	Callsign   [8]byte  // Vehicle identifier (8 characters, valid characters are A-Z, 0-9, " " only).
	Capability uint8    // Max Aircraft Speed and ADS-B in capability
	Emitter    uint8    // Transmitting vehicle type.
	Alwencode  uint8    // Aircraft length and width encoding (table 2-35 of DO-282B). Upper Bound
	Gpslatoffs uint8    // GPS antenna lateral offset (table 2-36 of DO-282B).
	Gpslonoffs uint8    // GPS antenna longitudinal offset from nose [if non-zero, take position (in meters) divide by 2 and add one with max 60m] (table 2-37 DO-282B).
}

func (self *OwnShipStatic) MsgID() uint8 {
	return 201
}

func (self *OwnShipStatic) MsgName() string {
	return "OwnShipStatic"
}

func (self *OwnShipStatic) Pack(p *Packet) error {
	payload := make([]byte, 19)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Stallspeed))
	copy(payload[2:], self.Icao[:])
	payload[5] = byte(self.Integrity)
	copy(payload[6:], self.Callsign[:])
	payload[14] = byte(self.Capability)
	payload[15] = byte(self.Emitter)
	payload[16] = byte(self.Alwencode)
	payload[17] = byte(self.Gpslatoffs)
	payload[18] = byte(self.Gpslonoffs)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *OwnShipStatic) Unpack(p *Packet) error {
	if len(p.Payload) < 19 {
		return fmt.Errorf("payload too small")
	}
	self.Stallspeed = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	copy(self.Icao[:], p.Payload[2:5])
	self.Integrity = uint8(p.Payload[5])
	copy(self.Callsign[:], p.Payload[6:14])
	self.Capability = uint8(p.Payload[14])
	self.Emitter = uint8(p.Payload[15])
	self.Alwencode = uint8(p.Payload[16])
	self.Gpslatoffs = uint8(p.Payload[17])
	self.Gpslonoffs = uint8(p.Payload[18])
	return nil
}

// Message IDs
const (
	MSG_ID_TRAFFIC_REPORT   = 246
	MSG_ID_UAVIONIX_STATUS  = 203
	MSG_ID_OWN_SHIP_DYNAMIC = 202
	MSG_ID_OWN_SHIP_STATIC  = 201
)

// DialectUavionix is the dialect represented by uavionix.xml
var DialectUavionix *Dialect = &Dialect{
	Name: "uavionix",
	crcExtras: map[uint8]uint8{
		246: 184, // MSG_ID_TRAFFIC_REPORT
		203: 85,  // MSG_ID_UAVIONIX_STATUS
		202: 7,   // MSG_ID_OWN_SHIP_DYNAMIC
		201: 126, // MSG_ID_OWN_SHIP_STATIC
	},
}
