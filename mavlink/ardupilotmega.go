package mavlink

import (
	"encoding/binary"
	"fmt"
	"math"
)

//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

// MavCmd:
const (
	MAV_CMD_DO_MOTOR_TEST = 209 // Mission command to perform motor test
)

// LimitsState:
const (
	LIMITS_INIT       = 0 //  pre-initialization
	LIMITS_DISABLED   = 1 //  disabled
	LIMITS_ENABLED    = 2 //  checking limits
	LIMITS_TRIGGERED  = 3 //  a limit has been breached
	LIMITS_RECOVERING = 4 //  taking action eg. RTL
	LIMITS_RECOVERED  = 5 //  we're no longer in breach of a limit
)

// LimitModule:
const (
	LIMIT_GPSLOCK  = 1 //  pre-initialization
	LIMIT_GEOFENCE = 2 //  disabled
	LIMIT_ALTITUDE = 4 //  checking limits
)

// RallyFlags: Flags in RALLY_POINT message
const (
	FAVORABLE_WIND   = 1 // Flag set when requiring favorable winds for landing.
	LAND_IMMEDIATELY = 2 // Flag set when plane is to immediately descend to break altitude and land without GCS intervention.  Flag not set when plane is to loiter at Rally point until commanded to land.
)

// ParachuteAction:
const (
	PARACHUTE_DISABLE = 0 // Disable parachute release
	PARACHUTE_ENABLE  = 1 // Enable parachute release
	PARACHUTE_RELEASE = 2 // Release parachute
)

// MotorTestThrottleType:
const (
	MOTOR_TEST_THROTTLE_PERCENT = 0 // throttle as a percentage from 0 ~ 100
	MOTOR_TEST_THROTTLE_PWM     = 1 // throttle as an absolute PWM value (normally in range of 1000~2000)
	MOTOR_TEST_THROTTLE_PILOT   = 2 // throttle pass-through from pilot's transmitter
)

// CameraStatusTypes:
const (
	CAMERA_STATUS_TYPE_HEARTBEAT  = 0 // Camera heartbeat, announce camera component ID at 1hz
	CAMERA_STATUS_TYPE_TRIGGER    = 1 // Camera image triggered
	CAMERA_STATUS_TYPE_DISCONNECT = 2 // Camera connection lost
	CAMERA_STATUS_TYPE_ERROR      = 3 // Camera unknown error
	CAMERA_STATUS_TYPE_LOWBATT    = 4 // Camera battery low. Parameter p1 shows reported voltage
	CAMERA_STATUS_TYPE_LOWSTORE   = 5 // Camera storage low. Parameter p1 shows reported shots remaining
	CAMERA_STATUS_TYPE_LOWSTOREV  = 6 // Camera storage low. Parameter p1 shows reported video minutes remaining
)

// CameraFeedbackFlags:
const (
	VIDEO       = 1 // Shooting video, not stills
	BADEXPOSURE = 2 // Unable to achieve requested exposure (e.g. shutter speed too low)
	CLOSEDLOOP  = 3 // Closed loop feedback from camera, we know for sure it has successfully taken a picture
	OPENLOOP    = 4 // Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken a picture
)

// BevRequest:
const (
	BEV_REQUEST_NONE                 = 0  // Do nothing (default)
	BEV_REQUEST_GEAR_UP              = 1  // Raise gear
	BEV_REQUEST_GEAR_DOWN            = 2  // Lower gear
	BEV_REQUEST_GEAR_TOGGLE          = 3  // Toggle gear
	BEV_REQUEST_TRANSITION_TO_COPTER = 4  // Transition to copter
	BEV_REQUEST_TRANSITION_TO_PLANE  = 5  // Transition to plane
	BEV_REQUEST_TRANSITION_TOGGLE    = 6  // Toggle state
	BEV_REQUEST_SERVOS_TOGGLE        = 7  // Toggle servos (on aux3 and 4)
	BEV_REQUEST_EPM_GRAB             = 8  // Signal the EPM to grab
	BEV_REQUEST_EPM_RELEASE          = 9  // Signal the EPM to release
	BEV_REQUEST_EPM_TOGGLE           = 10 // Signal the EPM to toggle state
)

// BevStatusTransition:
const (
	BEV_STATUS_TRANSITION_TO_COPTER   = 0 // Transitioning to copter
	BEV_STATUS_TRANSITION_TO_PLANE    = 1 // Transitioning to plane
	BEV_STATUS_TRANSITION_FULL_COPTER = 2 // Not transitioning, full copter
	BEV_STATUS_TRANSITION_FULL_PLANE  = 3 // Not transitioning, full plane
)

// BevStatusGear:
const (
	BEV_STATUS_GEAR_UP   = 0 // Gear is up
	BEV_STATUS_GEAR_DOWN = 1 // Gear is down
)

// BevReg1Devices:
const (
	BEV_REG1_DEVICE_NONE             = 0  // Default value
	BEV_REG1_DEVICE_CAMERATRIGGER    = 1  // Mapping camera trigger
	BEV_REG1_DEVICE_GIMBAL3          = 2  // 3 axis gimbal
	BEV_REG1_DEVICE_REDEDGE          = 4  // RedEdge Camera
	BEV_REG1_DEVICE_EPMGRIPPER       = 8  // EPM Cargo Gripper
	BEV_REG1_DEVICE_SENTERA          = 16 // Sentera Camera
	BEV_REG1_DEVICE_RTKCAMERATRIGGER = 32 // RTK Capable Camera
)

// BevReg2Devices:
const (
	BEV_REG2_DEVICE_NONE = 0 // Default value
)

// BevReg3Devices:
const (
	BEV_REG3_DEVICE_NONE = 0 // Default value
)

// BevReg4Devices:
const (
	BEV_REG4_DEVICE_NONE = 0 // Default value
)

// Offsets and calibrations values for hardware
//         sensors. This makes it easier to debug the calibration process.
type SensorOffsets struct {
	MagDeclination float32 // magnetic declination (radians)
	RawPress       int32   // raw pressure from barometer
	RawTemp        int32   // raw temperature from barometer
	GyroCalX       float32 // gyro X calibration
	GyroCalY       float32 // gyro Y calibration
	GyroCalZ       float32 // gyro Z calibration
	AccelCalX      float32 // accel X calibration
	AccelCalY      float32 // accel Y calibration
	AccelCalZ      float32 // accel Z calibration
	MagOfsX        int16   // magnetometer X offset
	MagOfsY        int16   // magnetometer Y offset
	MagOfsZ        int16   // magnetometer Z offset
}

func (self *SensorOffsets) MsgID() uint8 {
	return 150
}

func (self *SensorOffsets) MsgName() string {
	return "SensorOffsets"
}

func (self *SensorOffsets) Pack(p *Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.MagDeclination))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.RawPress))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.RawTemp))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.GyroCalX))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.GyroCalY))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.GyroCalZ))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.AccelCalX))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.AccelCalY))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.AccelCalZ))
	binary.LittleEndian.PutUint16(payload[36:], uint16(self.MagOfsX))
	binary.LittleEndian.PutUint16(payload[38:], uint16(self.MagOfsY))
	binary.LittleEndian.PutUint16(payload[40:], uint16(self.MagOfsZ))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SensorOffsets) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	self.MagDeclination = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.RawPress = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.RawTemp = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.GyroCalX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.GyroCalY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.GyroCalZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.AccelCalX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.AccelCalY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.AccelCalZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.MagOfsX = int16(binary.LittleEndian.Uint16(p.Payload[36:]))
	self.MagOfsY = int16(binary.LittleEndian.Uint16(p.Payload[38:]))
	self.MagOfsZ = int16(binary.LittleEndian.Uint16(p.Payload[40:]))
	return nil
}

// Deprecated. Use MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS instead. Set the magnetometer offsets
type SetMagOffsets struct {
	MagOfsX         int16 // magnetometer X offset
	MagOfsY         int16 // magnetometer Y offset
	MagOfsZ         int16 // magnetometer Z offset
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *SetMagOffsets) MsgID() uint8 {
	return 151
}

func (self *SetMagOffsets) MsgName() string {
	return "SetMagOffsets"
}

func (self *SetMagOffsets) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.MagOfsX))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.MagOfsY))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.MagOfsZ))
	payload[6] = byte(self.TargetSystem)
	payload[7] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetMagOffsets) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.MagOfsX = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.MagOfsY = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.MagOfsZ = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.TargetSystem = uint8(p.Payload[6])
	self.TargetComponent = uint8(p.Payload[7])
	return nil
}

// state of APM memory
type Meminfo struct {
	Brkval  uint16 // heap top
	Freemem uint16 // free memory
}

func (self *Meminfo) MsgID() uint8 {
	return 152
}

func (self *Meminfo) MsgName() string {
	return "Meminfo"
}

func (self *Meminfo) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Brkval))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Freemem))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Meminfo) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.Brkval = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Freemem = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	return nil
}

// raw ADC output
type ApAdc struct {
	Adc1 uint16 // ADC output 1
	Adc2 uint16 // ADC output 2
	Adc3 uint16 // ADC output 3
	Adc4 uint16 // ADC output 4
	Adc5 uint16 // ADC output 5
	Adc6 uint16 // ADC output 6
}

func (self *ApAdc) MsgID() uint8 {
	return 153
}

func (self *ApAdc) MsgName() string {
	return "ApAdc"
}

func (self *ApAdc) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Adc1))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Adc2))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Adc3))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Adc4))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Adc5))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Adc6))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ApAdc) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Adc1 = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Adc2 = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.Adc3 = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Adc4 = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Adc5 = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Adc6 = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	return nil
}

// Configure on-board Camera Control System.
type DigicamConfigure struct {
	ExtraValue      float32 // Correspondent value to given extra_param
	ShutterSpeed    uint16  // Divisor number //e.g. 1000 means 1/1000 (0 means ignore)
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Mode            uint8   // Mode enumeration from 1 to N //P, TV, AV, M, Etc (0 means ignore)
	Aperture        uint8   // F stop number x 10 //e.g. 28 means 2.8 (0 means ignore)
	Iso             uint8   // ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore)
	ExposureType    uint8   // Exposure type enumeration from 1 to N (0 means ignore)
	CommandId       uint8   // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
	EngineCutOff    uint8   // Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore)
}

func (self *DigicamConfigure) MsgID() uint8 {
	return 154
}

func (self *DigicamConfigure) MsgName() string {
	return "DigicamConfigure"
}

func (self *DigicamConfigure) Pack(p *Packet) error {
	payload := make([]byte, 15)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.ExtraValue))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.ShutterSpeed))
	payload[6] = byte(self.TargetSystem)
	payload[7] = byte(self.TargetComponent)
	payload[8] = byte(self.Mode)
	payload[9] = byte(self.Aperture)
	payload[10] = byte(self.Iso)
	payload[11] = byte(self.ExposureType)
	payload[12] = byte(self.CommandId)
	payload[13] = byte(self.EngineCutOff)
	payload[14] = byte(self.ExtraParam)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *DigicamConfigure) Unpack(p *Packet) error {
	if len(p.Payload) < 15 {
		return fmt.Errorf("payload too small")
	}
	self.ExtraValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.ShutterSpeed = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.TargetSystem = uint8(p.Payload[6])
	self.TargetComponent = uint8(p.Payload[7])
	self.Mode = uint8(p.Payload[8])
	self.Aperture = uint8(p.Payload[9])
	self.Iso = uint8(p.Payload[10])
	self.ExposureType = uint8(p.Payload[11])
	self.CommandId = uint8(p.Payload[12])
	self.EngineCutOff = uint8(p.Payload[13])
	self.ExtraParam = uint8(p.Payload[14])
	return nil
}

// Control on-board Camera Control System to take shots.
type DigicamControl struct {
	ExtraValue      float32 // Correspondent value to given extra_param
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Session         uint8   // 0: stop, 1: start or keep it up //Session control e.g. show/hide lens
	ZoomPos         uint8   // 1 to N //Zoom's absolute position (0 means ignore)
	ZoomStep        int8    // -100 to 100 //Zooming step value to offset zoom from the current position
	FocusLock       uint8   // 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
	Shot            uint8   // 0: ignore, 1: shot or start filming
	CommandId       uint8   // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore)
}

func (self *DigicamControl) MsgID() uint8 {
	return 155
}

func (self *DigicamControl) MsgName() string {
	return "DigicamControl"
}

func (self *DigicamControl) Pack(p *Packet) error {
	payload := make([]byte, 13)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.ExtraValue))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)
	payload[6] = byte(self.Session)
	payload[7] = byte(self.ZoomPos)
	payload[8] = byte(self.ZoomStep)
	payload[9] = byte(self.FocusLock)
	payload[10] = byte(self.Shot)
	payload[11] = byte(self.CommandId)
	payload[12] = byte(self.ExtraParam)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *DigicamControl) Unpack(p *Packet) error {
	if len(p.Payload) < 13 {
		return fmt.Errorf("payload too small")
	}
	self.ExtraValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	self.Session = uint8(p.Payload[6])
	self.ZoomPos = uint8(p.Payload[7])
	self.ZoomStep = int8(p.Payload[8])
	self.FocusLock = uint8(p.Payload[9])
	self.Shot = uint8(p.Payload[10])
	self.CommandId = uint8(p.Payload[11])
	self.ExtraParam = uint8(p.Payload[12])
	return nil
}

// Message to configure a camera mount, directional antenna, etc.
type MountConfigure struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	MountMode       uint8 // mount operating mode (see MAV_MOUNT_MODE enum)
	StabRoll        uint8 // (1 = yes, 0 = no)
	StabPitch       uint8 // (1 = yes, 0 = no)
	StabYaw         uint8 // (1 = yes, 0 = no)
}

func (self *MountConfigure) MsgID() uint8 {
	return 156
}

func (self *MountConfigure) MsgName() string {
	return "MountConfigure"
}

func (self *MountConfigure) Pack(p *Packet) error {
	payload := make([]byte, 6)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.MountMode)
	payload[3] = byte(self.StabRoll)
	payload[4] = byte(self.StabPitch)
	payload[5] = byte(self.StabYaw)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MountConfigure) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.MountMode = uint8(p.Payload[2])
	self.StabRoll = uint8(p.Payload[3])
	self.StabPitch = uint8(p.Payload[4])
	self.StabYaw = uint8(p.Payload[5])
	return nil
}

// Message to control a camera mount, directional antenna, etc.
type MountControl struct {
	InputA          int32 // pitch(deg*100) or lat, depending on mount mode
	InputB          int32 // roll(deg*100) or lon depending on mount mode
	InputC          int32 // yaw(deg*100) or alt (in cm) depending on mount mode
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	SavePosition    uint8 // if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
}

func (self *MountControl) MsgID() uint8 {
	return 157
}

func (self *MountControl) MsgName() string {
	return "MountControl"
}

func (self *MountControl) Pack(p *Packet) error {
	payload := make([]byte, 15)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.InputA))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.InputB))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.InputC))
	payload[12] = byte(self.TargetSystem)
	payload[13] = byte(self.TargetComponent)
	payload[14] = byte(self.SavePosition)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MountControl) Unpack(p *Packet) error {
	if len(p.Payload) < 15 {
		return fmt.Errorf("payload too small")
	}
	self.InputA = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.InputB = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.InputC = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.TargetSystem = uint8(p.Payload[12])
	self.TargetComponent = uint8(p.Payload[13])
	self.SavePosition = uint8(p.Payload[14])
	return nil
}

// Message with some status from APM to GCS about camera or antenna mount
type MountStatus struct {
	PointingA       int32 // pitch(deg*100) or lat, depending on mount mode
	PointingB       int32 // roll(deg*100) or lon depending on mount mode
	PointingC       int32 // yaw(deg*100) or alt (in cm) depending on mount mode
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *MountStatus) MsgID() uint8 {
	return 158
}

func (self *MountStatus) MsgName() string {
	return "MountStatus"
}

func (self *MountStatus) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.PointingA))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.PointingB))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.PointingC))
	payload[12] = byte(self.TargetSystem)
	payload[13] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MountStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.PointingA = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.PointingB = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.PointingC = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.TargetSystem = uint8(p.Payload[12])
	self.TargetComponent = uint8(p.Payload[13])
	return nil
}

// A fence point. Used to set a point when from
// 	      GCS -> MAV. Also used to return a point from MAV -> GCS
type FencePoint struct {
	Lat             float32 // Latitude of point
	Lng             float32 // Longitude of point
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Idx             uint8   // point index (first point is 1, 0 is for return point)
	Count           uint8   // total number of points (for sanity checking)
}

func (self *FencePoint) MsgID() uint8 {
	return 160
}

func (self *FencePoint) MsgName() string {
	return "FencePoint"
}

func (self *FencePoint) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Lat))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Lng))
	payload[8] = byte(self.TargetSystem)
	payload[9] = byte(self.TargetComponent)
	payload[10] = byte(self.Idx)
	payload[11] = byte(self.Count)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FencePoint) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Lat = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lng = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.TargetSystem = uint8(p.Payload[8])
	self.TargetComponent = uint8(p.Payload[9])
	self.Idx = uint8(p.Payload[10])
	self.Count = uint8(p.Payload[11])
	return nil
}

// Request a current fence point from MAV
type FenceFetchPoint struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Idx             uint8 // point index (first point is 1, 0 is for return point)
}

func (self *FenceFetchPoint) MsgID() uint8 {
	return 161
}

func (self *FenceFetchPoint) MsgName() string {
	return "FenceFetchPoint"
}

func (self *FenceFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.Idx)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FenceFetchPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.Idx = uint8(p.Payload[2])
	return nil
}

// Status of geo-fencing. Sent in extended
// 	    status stream when fencing enabled
type FenceStatus struct {
	BreachTime   uint32 // time of last breach in milliseconds since boot
	BreachCount  uint16 // number of fence breaches
	BreachStatus uint8  // 0 if currently inside fence, 1 if outside
	BreachType   uint8  // last breach type (see FENCE_BREACH_* enum)
}

func (self *FenceStatus) MsgID() uint8 {
	return 162
}

func (self *FenceStatus) MsgName() string {
	return "FenceStatus"
}

func (self *FenceStatus) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.BreachTime))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.BreachCount))
	payload[6] = byte(self.BreachStatus)
	payload[7] = byte(self.BreachType)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FenceStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.BreachTime = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.BreachCount = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.BreachStatus = uint8(p.Payload[6])
	self.BreachType = uint8(p.Payload[7])
	return nil
}

// Status of DCM attitude estimator
type Ahrs struct {
	Omegaix     float32 // X gyro drift estimate rad/s
	Omegaiy     float32 // Y gyro drift estimate rad/s
	Omegaiz     float32 // Z gyro drift estimate rad/s
	AccelWeight float32 // average accel_weight
	RenormVal   float32 // average renormalisation value
	ErrorRp     float32 // average error_roll_pitch value
	ErrorYaw    float32 // average error_yaw value
}

func (self *Ahrs) MsgID() uint8 {
	return 163
}

func (self *Ahrs) MsgName() string {
	return "Ahrs"
}

func (self *Ahrs) Pack(p *Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Omegaix))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Omegaiy))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Omegaiz))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.AccelWeight))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.RenormVal))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.ErrorRp))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.ErrorYaw))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Ahrs) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	self.Omegaix = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Omegaiy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Omegaiz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.AccelWeight = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.RenormVal = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.ErrorRp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.ErrorYaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

// Status of simulation environment, if used
type Simstate struct {
	Roll  float32 // Roll angle (rad)
	Pitch float32 // Pitch angle (rad)
	Yaw   float32 // Yaw angle (rad)
	Xacc  float32 // X acceleration m/s/s
	Yacc  float32 // Y acceleration m/s/s
	Zacc  float32 // Z acceleration m/s/s
	Xgyro float32 // Angular speed around X axis rad/s
	Ygyro float32 // Angular speed around Y axis rad/s
	Zgyro float32 // Angular speed around Z axis rad/s
	Lat   int32   // Latitude in degrees * 1E7
	Lng   int32   // Longitude in degrees * 1E7
}

func (self *Simstate) MsgID() uint8 {
	return 164
}

func (self *Simstate) MsgName() string {
	return "Simstate"
}

func (self *Simstate) Pack(p *Packet) error {
	payload := make([]byte, 44)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Xacc))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Yacc))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Zacc))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Xgyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Ygyro))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Zgyro))
	binary.LittleEndian.PutUint32(payload[36:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[40:], uint32(self.Lng))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Simstate) Unpack(p *Packet) error {
	if len(p.Payload) < 44 {
		return fmt.Errorf("payload too small")
	}
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[40:]))
	return nil
}

// Status of key hardware
type Hwstatus struct {
	Vcc    uint16 // board voltage (mV)
	I2cerr uint8  // I2C error count
}

func (self *Hwstatus) MsgID() uint8 {
	return 165
}

func (self *Hwstatus) MsgName() string {
	return "Hwstatus"
}

func (self *Hwstatus) Pack(p *Packet) error {
	payload := make([]byte, 3)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Vcc))
	payload[2] = byte(self.I2cerr)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Hwstatus) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.Vcc = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.I2cerr = uint8(p.Payload[2])
	return nil
}

// Status generated by radio
type Radio struct {
	Rxerrors uint16 // receive errors
	Fixed    uint16 // count of error corrected packets
	Rssi     uint8  // local signal strength
	Remrssi  uint8  // remote signal strength
	Txbuf    uint8  // how full the tx buffer is as a percentage
	Noise    uint8  // background noise level
	Remnoise uint8  // remote background noise level
}

func (self *Radio) MsgID() uint8 {
	return 166
}

func (self *Radio) MsgName() string {
	return "Radio"
}

func (self *Radio) Pack(p *Packet) error {
	payload := make([]byte, 9)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Rxerrors))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Fixed))
	payload[4] = byte(self.Rssi)
	payload[5] = byte(self.Remrssi)
	payload[6] = byte(self.Txbuf)
	payload[7] = byte(self.Noise)
	payload[8] = byte(self.Remnoise)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Radio) Unpack(p *Packet) error {
	if len(p.Payload) < 9 {
		return fmt.Errorf("payload too small")
	}
	self.Rxerrors = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Fixed = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.Rssi = uint8(p.Payload[4])
	self.Remrssi = uint8(p.Payload[5])
	self.Txbuf = uint8(p.Payload[6])
	self.Noise = uint8(p.Payload[7])
	self.Remnoise = uint8(p.Payload[8])
	return nil
}

// Status of AP_Limits. Sent in extended
// 	    status stream when AP_Limits is enabled
type LimitsStatus struct {
	LastTrigger   uint32 // time of last breach in milliseconds since boot
	LastAction    uint32 // time of last recovery action in milliseconds since boot
	LastRecovery  uint32 // time of last successful recovery in milliseconds since boot
	LastClear     uint32 // time of last all-clear in milliseconds since boot
	BreachCount   uint16 // number of fence breaches
	LimitsState   uint8  // state of AP_Limits, (see enum LimitState, LIMITS_STATE)
	ModsEnabled   uint8  // AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
	ModsRequired  uint8  // AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
	ModsTriggered uint8  // AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
}

func (self *LimitsStatus) MsgID() uint8 {
	return 167
}

func (self *LimitsStatus) MsgName() string {
	return "LimitsStatus"
}

func (self *LimitsStatus) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.LastTrigger))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.LastAction))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.LastRecovery))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.LastClear))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.BreachCount))
	payload[18] = byte(self.LimitsState)
	payload[19] = byte(self.ModsEnabled)
	payload[20] = byte(self.ModsRequired)
	payload[21] = byte(self.ModsTriggered)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LimitsStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.LastTrigger = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.LastAction = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.LastRecovery = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.LastClear = uint32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.BreachCount = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.LimitsState = uint8(p.Payload[18])
	self.ModsEnabled = uint8(p.Payload[19])
	self.ModsRequired = uint8(p.Payload[20])
	self.ModsTriggered = uint8(p.Payload[21])
	return nil
}

// Wind estimation
type Wind struct {
	Direction float32 // wind direction that wind is coming from (degrees)
	Speed     float32 // wind speed in ground plane (m/s)
	SpeedZ    float32 // vertical wind speed (m/s)
}

func (self *Wind) MsgID() uint8 {
	return 168
}

func (self *Wind) MsgName() string {
	return "Wind"
}

func (self *Wind) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Direction))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Speed))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.SpeedZ))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Wind) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Direction = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Speed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SpeedZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	return nil
}

// Data packet, size 16
type Data16 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [16]uint8 // raw data
}

func (self *Data16) MsgID() uint8 {
	return 169
}

func (self *Data16) MsgName() string {
	return "Data16"
}

func (self *Data16) Pack(p *Packet) error {
	payload := make([]byte, 18)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data16) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:18])
	return nil
}

// Data packet, size 32
type Data32 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [32]uint8 // raw data
}

func (self *Data32) MsgID() uint8 {
	return 170
}

func (self *Data32) MsgName() string {
	return "Data32"
}

func (self *Data32) Pack(p *Packet) error {
	payload := make([]byte, 34)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data32) Unpack(p *Packet) error {
	if len(p.Payload) < 34 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:34])
	return nil
}

// Data packet, size 64
type Data64 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [64]uint8 // raw data
}

func (self *Data64) MsgID() uint8 {
	return 171
}

func (self *Data64) MsgName() string {
	return "Data64"
}

func (self *Data64) Pack(p *Packet) error {
	payload := make([]byte, 66)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data64) Unpack(p *Packet) error {
	if len(p.Payload) < 66 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:66])
	return nil
}

// Data packet, size 96
type Data96 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [96]uint8 // raw data
}

func (self *Data96) MsgID() uint8 {
	return 172
}

func (self *Data96) MsgName() string {
	return "Data96"
}

func (self *Data96) Pack(p *Packet) error {
	payload := make([]byte, 98)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data96) Unpack(p *Packet) error {
	if len(p.Payload) < 98 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:98])
	return nil
}

// Rangefinder reporting
type Rangefinder struct {
	Distance float32 // distance in meters
	Voltage  float32 // raw voltage if available, zero otherwise
}

func (self *Rangefinder) MsgID() uint8 {
	return 173
}

func (self *Rangefinder) MsgName() string {
	return "Rangefinder"
}

func (self *Rangefinder) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Distance))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Voltage))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Rangefinder) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.Distance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Voltage = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	return nil
}

// Airspeed auto-calibration
type AirspeedAutocal struct {
	Vx           float32 // GPS velocity north m/s
	Vy           float32 // GPS velocity east m/s
	Vz           float32 // GPS velocity down m/s
	DiffPressure float32 // Differential pressure pascals
	Eas2tas      float32 // Estimated to true airspeed ratio
	Ratio        float32 // Airspeed ratio
	StateX       float32 // EKF state x
	StateY       float32 // EKF state y
	StateZ       float32 // EKF state z
	Pax          float32 // EKF Pax
	Pby          float32 // EKF Pby
	Pcz          float32 // EKF Pcz
}

func (self *AirspeedAutocal) MsgID() uint8 {
	return 174
}

func (self *AirspeedAutocal) MsgName() string {
	return "AirspeedAutocal"
}

func (self *AirspeedAutocal) Pack(p *Packet) error {
	payload := make([]byte, 48)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Vx))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Vy))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Vz))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.DiffPressure))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Eas2tas))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Ratio))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.StateX))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.StateY))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.StateZ))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Pax))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Pby))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.Pcz))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AirspeedAutocal) Unpack(p *Packet) error {
	if len(p.Payload) < 48 {
		return fmt.Errorf("payload too small")
	}
	self.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.DiffPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Eas2tas = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Ratio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.StateX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.StateY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.StateZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Pax = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Pby = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.Pcz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	return nil
}

// A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS
type RallyPoint struct {
	Lat             int32  // Latitude of point in degrees * 1E7
	Lng             int32  // Longitude of point in degrees * 1E7
	Alt             int16  // Transit / loiter altitude in meters relative to home
	BreakAlt        int16  // Break altitude in meters relative to home
	LandDir         uint16 // Heading to aim for when landing. In centi-degrees.
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	Idx             uint8  // point index (first point is 0)
	Count           uint8  // total number of points (for sanity checking)
	Flags           uint8  // See RALLY_FLAGS enum for definition of the bitmask.
}

func (self *RallyPoint) MsgID() uint8 {
	return 175
}

func (self *RallyPoint) MsgName() string {
	return "RallyPoint"
}

func (self *RallyPoint) Pack(p *Packet) error {
	payload := make([]byte, 19)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Lng))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Alt))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.BreakAlt))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.LandDir))
	payload[14] = byte(self.TargetSystem)
	payload[15] = byte(self.TargetComponent)
	payload[16] = byte(self.Idx)
	payload[17] = byte(self.Count)
	payload[18] = byte(self.Flags)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RallyPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 19 {
		return fmt.Errorf("payload too small")
	}
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Alt = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.BreakAlt = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.LandDir = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.TargetSystem = uint8(p.Payload[14])
	self.TargetComponent = uint8(p.Payload[15])
	self.Idx = uint8(p.Payload[16])
	self.Count = uint8(p.Payload[17])
	self.Flags = uint8(p.Payload[18])
	return nil
}

// Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not respond if the request is invalid.
type RallyFetchPoint struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Idx             uint8 // point index (first point is 0)
}

func (self *RallyFetchPoint) MsgID() uint8 {
	return 176
}

func (self *RallyFetchPoint) MsgName() string {
	return "RallyFetchPoint"
}

func (self *RallyFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.Idx)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RallyFetchPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.Idx = uint8(p.Payload[2])
	return nil
}

// Status of compassmot calibration
type CompassmotStatus struct {
	Current       float32 // current (amps)
	Compensationx float32 // Motor Compensation X
	Compensationy float32 // Motor Compensation Y
	Compensationz float32 // Motor Compensation Z
	Throttle      uint16  // throttle (percent*10)
	Interference  uint16  // interference (percent)
}

func (self *CompassmotStatus) MsgID() uint8 {
	return 177
}

func (self *CompassmotStatus) MsgName() string {
	return "CompassmotStatus"
}

func (self *CompassmotStatus) Pack(p *Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Current))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Compensationx))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Compensationy))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Compensationz))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Throttle))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Interference))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CompassmotStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	self.Current = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Compensationx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Compensationy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Compensationz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Throttle = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Interference = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	return nil
}

// Status of secondary AHRS filter if available
type Ahrs2 struct {
	Roll     float32 // Roll angle (rad)
	Pitch    float32 // Pitch angle (rad)
	Yaw      float32 // Yaw angle (rad)
	Altitude float32 // Altitude (MSL)
	Lat      int32   // Latitude in degrees * 1E7
	Lng      int32   // Longitude in degrees * 1E7
}

func (self *Ahrs2) MsgID() uint8 {
	return 178
}

func (self *Ahrs2) MsgName() string {
	return "Ahrs2"
}

func (self *Ahrs2) Pack(p *Packet) error {
	payload := make([]byte, 24)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Altitude))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Lng))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Ahrs2) Unpack(p *Packet) error {
	if len(p.Payload) < 24 {
		return fmt.Errorf("payload too small")
	}
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	return nil
}

// Camera Event
type CameraStatus struct {
	TimeUsec     uint64  // Image timestamp (microseconds since UNIX epoch, according to camera clock)
	P1           float32 // Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P2           float32 // Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P3           float32 // Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P4           float32 // Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	ImgIdx       uint16  // Image index
	TargetSystem uint8   // System ID
	CamIdx       uint8   // Camera ID
	EventId      uint8   // See CAMERA_STATUS_TYPES enum for definition of the bitmask
}

func (self *CameraStatus) MsgID() uint8 {
	return 179
}

func (self *CameraStatus) MsgName() string {
	return "CameraStatus"
}

func (self *CameraStatus) Pack(p *Packet) error {
	payload := make([]byte, 29)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.P1))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.P2))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.P3))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.P4))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.ImgIdx))
	payload[26] = byte(self.TargetSystem)
	payload[27] = byte(self.CamIdx)
	payload[28] = byte(self.EventId)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CameraStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 29 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.P1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.P2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.P3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.P4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.ImgIdx = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.TargetSystem = uint8(p.Payload[26])
	self.CamIdx = uint8(p.Payload[27])
	self.EventId = uint8(p.Payload[28])
	return nil
}

// Camera Capture Feedback
type CameraFeedback struct {
	TimeUsec     uint64  // Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB)
	Lat          int32   // Latitude in (deg * 1E7)
	Lng          int32   // Longitude in (deg * 1E7)
	AltMsl       float32 // Altitude Absolute (meters AMSL)
	AltRel       float32 // Altitude Relative (meters above HOME location)
	Roll         float32 // Camera Roll angle (earth frame, degrees, +-180)
	Pitch        float32 // Camera Pitch angle (earth frame, degrees, +-180)
	Yaw          float32 // Camera Yaw (earth frame, degrees, 0-360, true)
	FocLen       float32 // Focal Length (mm)
	ImgIdx       uint16  // Image index
	TargetSystem uint8   // System ID
	CamIdx       uint8   // Camera ID
	Flags        uint8   // See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
}

func (self *CameraFeedback) MsgID() uint8 {
	return 180
}

func (self *CameraFeedback) MsgName() string {
	return "CameraFeedback"
}

func (self *CameraFeedback) Pack(p *Packet) error {
	payload := make([]byte, 45)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Lng))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.AltMsl))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.AltRel))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.FocLen))
	binary.LittleEndian.PutUint16(payload[40:], uint16(self.ImgIdx))
	payload[42] = byte(self.TargetSystem)
	payload[43] = byte(self.CamIdx)
	payload[44] = byte(self.Flags)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CameraFeedback) Unpack(p *Packet) error {
	if len(p.Payload) < 45 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.AltMsl = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.AltRel = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.FocLen = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.ImgIdx = uint16(binary.LittleEndian.Uint16(p.Payload[40:]))
	self.TargetSystem = uint8(p.Payload[42])
	self.CamIdx = uint8(p.Payload[43])
	self.Flags = uint8(p.Payload[44])
	return nil
}

// 2nd Battery status
type Battery2 struct {
	Voltage        uint16 // voltage in millivolts
	CurrentBattery int16  // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
}

func (self *Battery2) MsgID() uint8 {
	return 181
}

func (self *Battery2) MsgName() string {
	return "Battery2"
}

func (self *Battery2) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Voltage))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.CurrentBattery))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Battery2) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.Voltage = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.CurrentBattery = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	return nil
}

// Requests specific to Fire Fly 6
type BevRequest struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Request         uint8 // request from BEV_REQUEST enum
}

func (self *BevRequest) MsgID() uint8 {
	return 182
}

func (self *BevRequest) MsgName() string {
	return "BevRequest"
}

func (self *BevRequest) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.Request)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *BevRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.Request = uint8(p.Payload[2])
	return nil
}

// Report transition and gear status from FireFLY6 to ground station. See BEV_STATUS* enums
type BevStatus struct {
	GpsRtkHaccMm  uint32 // RTK horizontal accuracy, 0 if no RTK
	GpsRtkVaccMm  uint32 // RTK vertical accuracy, 0 if no RTK
	TriggerCount  uint16 // Numer of times camera has been triggered
	GimbalPitchCd int16  // Camera gimbal pitch displacement from horizon [-17999 18000], 0 if no gimbal
	GimbalYawCd   int16  // Camera gimbal yaw from true north [-17999 18000], 0 if no gimbal
	Transition    uint8  // A value from BEV_STATUS_TRANSITION enum
	Gear          uint8  // A value from BEV_STATUS_GEAR enum
}

func (self *BevStatus) MsgID() uint8 {
	return 183
}

func (self *BevStatus) MsgName() string {
	return "BevStatus"
}

func (self *BevStatus) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.GpsRtkHaccMm))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.GpsRtkVaccMm))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.TriggerCount))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.GimbalPitchCd))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.GimbalYawCd))
	payload[14] = byte(self.Transition)
	payload[15] = byte(self.Gear)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *BevStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 16 {
		return fmt.Errorf("payload too small")
	}
	self.GpsRtkHaccMm = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.GpsRtkVaccMm = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.TriggerCount = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.GimbalPitchCd = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.GimbalYawCd = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Transition = uint8(p.Payload[14])
	self.Gear = uint8(p.Payload[15])
	return nil
}

// Request from GCS to AP to adjust camera position
type BevGimbalSpeed struct {
	PitchRate       int16 // Pitch rate in degx100
	YawRate         int16 // Yaw rate in degx100
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *BevGimbalSpeed) MsgID() uint8 {
	return 184
}

func (self *BevGimbalSpeed) MsgName() string {
	return "BevGimbalSpeed"
}

func (self *BevGimbalSpeed) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.PitchRate))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.YawRate))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *BevGimbalSpeed) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.PitchRate = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.YawRate = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	return nil
}

// Reports which I2C payloads are attached. See BEV_REG#_* enums
type BevPayloadReport struct {
	Register1 uint32 // Bitmask 1 of attached devices
	Register2 uint32 // Bitmask 2 of attached devices
	Register3 uint32 // Bitmask 3 of attached devices
	Register4 uint32 // Bitmask 4 of attached devices
}

func (self *BevPayloadReport) MsgID() uint8 {
	return 185
}

func (self *BevPayloadReport) MsgName() string {
	return "BevPayloadReport"
}

func (self *BevPayloadReport) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Register1))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Register2))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Register3))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Register4))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *BevPayloadReport) Unpack(p *Packet) error {
	if len(p.Payload) < 16 {
		return fmt.Errorf("payload too small")
	}
	self.Register1 = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Register2 = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Register3 = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Register4 = uint32(binary.LittleEndian.Uint32(p.Payload[12:]))
	return nil
}

// Message IDs
const (
	MSG_ID_SENSOR_OFFSETS     = 150
	MSG_ID_SET_MAG_OFFSETS    = 151
	MSG_ID_MEMINFO            = 152
	MSG_ID_AP_ADC             = 153
	MSG_ID_DIGICAM_CONFIGURE  = 154
	MSG_ID_DIGICAM_CONTROL    = 155
	MSG_ID_MOUNT_CONFIGURE    = 156
	MSG_ID_MOUNT_CONTROL      = 157
	MSG_ID_MOUNT_STATUS       = 158
	MSG_ID_FENCE_POINT        = 160
	MSG_ID_FENCE_FETCH_POINT  = 161
	MSG_ID_FENCE_STATUS       = 162
	MSG_ID_AHRS               = 163
	MSG_ID_SIMSTATE           = 164
	MSG_ID_HWSTATUS           = 165
	MSG_ID_RADIO              = 166
	MSG_ID_LIMITS_STATUS      = 167
	MSG_ID_WIND               = 168
	MSG_ID_DATA16             = 169
	MSG_ID_DATA32             = 170
	MSG_ID_DATA64             = 171
	MSG_ID_DATA96             = 172
	MSG_ID_RANGEFINDER        = 173
	MSG_ID_AIRSPEED_AUTOCAL   = 174
	MSG_ID_RALLY_POINT        = 175
	MSG_ID_RALLY_FETCH_POINT  = 176
	MSG_ID_COMPASSMOT_STATUS  = 177
	MSG_ID_AHRS2              = 178
	MSG_ID_CAMERA_STATUS      = 179
	MSG_ID_CAMERA_FEEDBACK    = 180
	MSG_ID_BATTERY2           = 181
	MSG_ID_BEV_REQUEST        = 182
	MSG_ID_BEV_STATUS         = 183
	MSG_ID_BEV_GIMBAL_SPEED   = 184
	MSG_ID_BEV_PAYLOAD_REPORT = 185
)

// DialectArdupilotmega is the dialect represented by ardupilotmega.xml
var DialectArdupilotmega *Dialect = &Dialect{
	Name: "ardupilotmega",
	crcExtras: map[uint8]uint8{
		150: 134, // MSG_ID_SENSOR_OFFSETS
		151: 219, // MSG_ID_SET_MAG_OFFSETS
		152: 208, // MSG_ID_MEMINFO
		153: 188, // MSG_ID_AP_ADC
		154: 84,  // MSG_ID_DIGICAM_CONFIGURE
		155: 22,  // MSG_ID_DIGICAM_CONTROL
		156: 19,  // MSG_ID_MOUNT_CONFIGURE
		157: 21,  // MSG_ID_MOUNT_CONTROL
		158: 134, // MSG_ID_MOUNT_STATUS
		160: 78,  // MSG_ID_FENCE_POINT
		161: 68,  // MSG_ID_FENCE_FETCH_POINT
		162: 189, // MSG_ID_FENCE_STATUS
		163: 127, // MSG_ID_AHRS
		164: 154, // MSG_ID_SIMSTATE
		165: 21,  // MSG_ID_HWSTATUS
		166: 21,  // MSG_ID_RADIO
		167: 144, // MSG_ID_LIMITS_STATUS
		168: 1,   // MSG_ID_WIND
		169: 234, // MSG_ID_DATA16
		170: 73,  // MSG_ID_DATA32
		171: 181, // MSG_ID_DATA64
		172: 22,  // MSG_ID_DATA96
		173: 83,  // MSG_ID_RANGEFINDER
		174: 167, // MSG_ID_AIRSPEED_AUTOCAL
		175: 138, // MSG_ID_RALLY_POINT
		176: 234, // MSG_ID_RALLY_FETCH_POINT
		177: 240, // MSG_ID_COMPASSMOT_STATUS
		178: 47,  // MSG_ID_AHRS2
		179: 189, // MSG_ID_CAMERA_STATUS
		180: 52,  // MSG_ID_CAMERA_FEEDBACK
		181: 174, // MSG_ID_BATTERY2
		182: 66,  // MSG_ID_BEV_REQUEST
		183: 73,  // MSG_ID_BEV_STATUS
		184: 68,  // MSG_ID_BEV_GIMBAL_SPEED
		185: 154, // MSG_ID_BEV_PAYLOAD_REPORT
	},
}
