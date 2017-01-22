package main

import (
	"cppinu"
	"flightTerms"
	"encoding/json"
	"fmt"
	"pilotMath"
	//"io/ioutil"
	"os"
)

const xOffset = -43.6189453125
const yOffset = 13.7592773437
const zOffset = -9.2390625

const homeLat = 37.9488
const homeLon = -122.048743
const START_DISTANCE_METERS = 30

const magneticDeclinationOffset = 13.8

const POP_FREQUENCY = 16
const SENSOR_FREQUENCY = 1600

const FOUR_SPACES = "    "

type sensorData struct {
	Timestamp float64 `json:"timestamp"`
	GpsLat        float64 `json:"gps_lat"`
	GpsLon        float64 `json:"gps_lon"`
	GpsAlt        float64 `json:"gps_alt"`
	Pitch         float32 `json:"pitch"`
	Yaw           float32 `json:"yaw"`
	Roll          float32 `json:"roll"`
	RelForwardAcc float32 `json:"rel_forward_acc"`
	RelUpAcc      float32 `json:"rel_up_acc"`
	AbsNorthAcc   float32 `json:"abs_north_acc"`
	AbsEastAcc    float32 `json:"abs_east_acc"`
	AbsUpAcc      float32 `json:"abs_up_acc"`
}

func appendJsonSerializableToFile(jsonEntity interface{}, filename string) {
	serialized, _ := json.MarshalIndent(jsonEntity, "", FOUR_SPACES)

	f, err := os.OpenFile(filename, os.O_APPEND|os.O_WRONLY, 0600)
	if err != nil {
		panic(err)
	}

	defer f.Close()

	if _, err = f.WriteString(fmt.Sprintf("%s\n", string(serialized))); err != nil {
		panic(err)
	}
}

func main() {
	home := flightTerms.GeoPoint {
		Latitude: homeLat,
		Longitude: homeLon,
	}
	shouldWrite := false

	counter := 0
	cppObj := cppinu.NewRawInertialNavigationReader(POP_FREQUENCY, SENSOR_FREQUENCY)
	var reader cppinu.SwigcptrRawInertialNavigationReader
	if safeCppObj, ok := cppObj.(cppinu.SwigcptrRawInertialNavigationReader); ok {
		reader = safeCppObj
	}
	for {
		reader.UpdateFromSensors(xOffset, yOffset, zOffset, magneticDeclinationOffset)
		if counter%(SENSOR_FREQUENCY/POP_FREQUENCY) == 0 {
			reader.MutateAbsoluteAcceleration(xOffset, yOffset, zOffset, magneticDeclinationOffset)
			pitch := reader.GetPitch()
			yaw := reader.GetYaw(magneticDeclinationOffset)
			roll := reader.GetRoll()
			relForwardAcc := reader.GetRelativeForwardAcceleration()
			relUpAcc := reader.GetRelativeUpAcceleration()
			absNorthAcc := reader.GetAbsoluteAccelerationNorth()
			absEastAcc := reader.GetAbsoluteAccelerationEast()
			absUpAcc := reader.GetAbsoluteAccelerationUp()

			gpsLat := 0.0
			gpsLon := 0.0
			gpsAlt := 0.0

			if reader.IsGPSReady() {
				gpsLat = reader.GetLatitude()
				gpsLon = reader.GetLongitude()
				gpsAlt = reader.GetGPSAltitude()
				distanceFromHome := flightTerms.GetDistance(
					home,
					flightTerms.GeoPoint{
						Latitude: gpsLat,
						Longitude: gpsLon,
					},
				)
				fmt.Printf("distance: %f\n", distanceFromHome)
				if distanceFromHome > START_DISTANCE_METERS {
					fmt.Printf("should write\n")
					shouldWrite = true
				} else {
					fmt.Printf("should not write\n")
					shouldWrite = false
				}
			}
			data := sensorData{
				Timestamp: float64(pilotMath.GetTimestampMS()),
				GpsLat:        gpsLat,
				GpsLon:        gpsLon,
				GpsAlt:        gpsAlt,
				Pitch:         pitch,
				Yaw:           yaw,
				Roll:          roll,
				RelForwardAcc: relForwardAcc,
				RelUpAcc:      relUpAcc,
				AbsNorthAcc:   absNorthAcc,
				AbsEastAcc:    absEastAcc,
				AbsUpAcc:      absUpAcc,
			}
			if shouldWrite {
				appendJsonSerializableToFile(data, "output.json")
			}
			// fmt.Printf("val of struct: %s\n", data)
			// fmt.Printf("P: %.1f, Y: %.1f, R: %.1f, ForwardA: %.3f, UpA: %.3f, abs N: %.3f, abs E: %.3f, abs Up: %.3f, lat: %f, lon: %f, alt: %f\n", pitch, yaw, roll, relForwardAcc, relUpAcc, absNorthAcc, absEastAcc, absUpAcc, gpsLat, gpsLon, gpsAlt);
		}
	}
}
