package flightTerms

import (
	"github.com/slobdell/byteSerialization"
	"github.com/slobdell/droneLanguage"
)

type Meters float64
type Degrees float64
type Percent float64
type MetersPerSecond float64
type DegreesPerSecond float64
type MilesPerHour float64
type Radians float64
type Seconds float64
type Hertz float64
type Timestamp float64

type GeoPoint struct {
    Latitude float64
    Longitude float64
}

type Velocity struct {
    LatitudeVelocity MetersPerSecond
    LongitudeVelocity MetersPerSecond
    AltitudeVelocity MetersPerSecond
}

type PositionalData struct {
    YawAzimuth *Degrees
    Roll *Degrees
    Pitch *Degrees
    Position *GeoPoint
    GroundAltitude *Meters
    RelativeAirAltitude *Meters
}

type DerivativeData struct {
    YawRate *DegreesPerSecond
    RollRate *DegreesPerSecond
    PitchRate *DegreesPerSecond
    Velocity *Velocity
}

type UpdateFrequencyData struct {
    Yaw Hertz
    Roll Hertz
    Pitch Hertz
    Position Hertz
    Altitude Hertz
}

type AircraftSensorPacket interface {
    GetPositionalData() PositionalData
    GetDerivativeData() DerivativeData
    GetUpdateFrequency() UpdateFrequencyData
	UpdateCompassOffsets(xOffset, yOffset, zOffset, magDeclinationOffset float32)
	StartCalibration()
	StopCalibration()
	IsCalibrated() bool
}

type SensorReader interface {
    GetSensorPacket() AircraftSensorPacket
}


type AircraftCommandPacket interface {
	GetTargetPitch() Degrees
	GetTargetYaw() Degrees
	GetTargetRoll() Degrees

	GetTargetPosition() GeoPoint

    GetPitchUpIntensity() Percent
    GetRollRightIntensity() Percent
    GetYawRightIntensity() Percent
    GetThrustIntensity() Percent
    GetThrustVectorAngle() Degrees
}

type PilotStrategy interface {
    GetNextMove(sensorPacket AircraftSensorPacket) AircraftCommandPacket
	HandleCommand(command byteSerialization.SimpleSerializable)
	HandleDestroy()
	Id() int
}

type AirFrame interface {
    UpdateMotorValues(commandPacket AircraftCommandPacket)
	UpdatePWMRanges(values droneLanguage.SetPWMRanges)
}
