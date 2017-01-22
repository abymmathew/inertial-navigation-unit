package main

import (
	"fmt"
	"encoding/json"
	"flightTerms"
	"pilotMath"
	"github.com/slobdell/basicMatrix"
	"io/ioutil"
)


func panicForError(e error) {
	if e != nil {
		panic(e)
	}
}

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

type sensorDataCollection []sensorData

func readFileAsJson(filename string, outputEntityAddress interface{}) {
	fileContents, err := ioutil.ReadFile(filename)
	panicForError(err)

	err = json.Unmarshal(fileContents, outputEntityAddress)
	panicForError(err)
}

type KalmanFilterGPSAccelerometer struct {
	H *basicMatrix.Matrix  // transformation matrix for input data
	P *basicMatrix.Matrix  // initial guess for covariance
	Q *basicMatrix.Matrix  // process (accelerometer) error variance
	R *basicMatrix.Matrix  // measurement (GPS) error variance
	currentState *basicMatrix.Matrix
	currentStateTimestampSeconds float64
}

func (k *KalmanFilterGPSAccelerometer) Predict(absAccNorth, absAccEast, absAccUp, timestampNow float64) {
	u := basicMatrix.NewMatrix(3, 1)

	u.Put(0, 0, absAccEast)
	u.Put(1, 0, absAccNorth)
	u.Put(2, 0, absAccUp)
	deltaT := timestampNow - k.currentStateTimestampSeconds

	B := k.newControlMatrix(deltaT)

	A := k.newStateTransitionMatrix(deltaT)
	updatedState := (A.MultipliedBy(k.currentState)).Add(B.MultipliedBy(u))
	k.currentState = updatedState

	updatedPredictionMatrix := ((A.MultipliedBy(k.P)).MultipliedBy(A.Transpose())).Add(k.Q)
	k.P = updatedPredictionMatrix
	k.currentStateTimestampSeconds = timestampNow
}

func (k *KalmanFilterGPSAccelerometer) Update(gpsLatitude, gpsLongitude, gpsAltitude float64) {
	positionLatitude := float64(pilotMath.LatitudeToMeters(gpsLatitude))
	positionLongitude := float64(pilotMath.LongitudeToMeters(gpsLongitude))

	prevLon := k.currentState.Get(0, 0)
	prevLat := k.currentState.Get(1, 0)
	prevAlt := k.currentState.Get(2, 0)

	vx := positionLongitude - prevLon
	vy := positionLatitude - prevLat
	vz := gpsAltitude - prevAlt
	
	z := basicMatrix.NewMatrix(6, 1)
	z.Put(0, 0, positionLongitude)
	z.Put(0, 1, positionLatitude)
	z.Put(0, 2, gpsAltitude)
	z.Put(0, 3, vx)
	z.Put(0, 4, vy)
	z.Put(0, 5, vz)

	y := z.Subtract(k.currentState)
	s := k.P.Add(k.R)
	sInverse, err := s.Inverse() 
	if err != nil {
		// matrix has no inverse, abort
		return
	}
	K := k.P.MultipliedBy(sInverse)
	fmt.Printf("Kalman gain: %f\n", K)

	updatedState := k.currentState.Add(K.MultipliedBy(y))
	k.currentState = updatedState

	// SBL is this supposed to be identity matrix or other P??
	updatedP := (basicMatrix.NewIdentityMatrix(6, 6).Subtract(K)).MultipliedBy(k.P)
	// updatedP := (k.P.Subtract(K)).MultipliedBy(k.P)
	k.P = updatedP
}

func (k *KalmanFilterGPSAccelerometer) newControlMatrix(deltaSeconds float64) *basicMatrix.Matrix {
	B := basicMatrix.NewMatrix(6, 3)
	dtSquared := 0.5 * deltaSeconds * deltaSeconds

	B.Put(0, 0, dtSquared)
	B.Put(0, 1, 0.0)
	B.Put(0, 2, 0.0)

	B.Put(1, 0, 0.0)
	B.Put(1, 1, dtSquared)
	B.Put(1, 2, 0.0)

	B.Put(2, 0, 0.0)
	B.Put(2, 1, 0.0)
	B.Put(2, 2, dtSquared)

	B.Put(3, 0, deltaSeconds)
	B.Put(3, 1, 0.0)
	B.Put(3, 2, 0.0)

	B.Put(4, 0, 0.0)
	B.Put(4, 1, deltaSeconds)
	B.Put(4, 2, 0.0)

	B.Put(5, 0, 0.0)
	B.Put(5, 1, 0.0)
	B.Put(5, 2, deltaSeconds)
	return B
}
func (k *KalmanFilterGPSAccelerometer) newStateTransitionMatrix(deltaSeconds float64) *basicMatrix.Matrix {
	A := basicMatrix.NewMatrix(6, 6)
	A.Put(0, 0, 1.0)
	A.Put(0, 1, 0.0)
	A.Put(0, 2, 0.0)
	A.Put(0, 3, deltaSeconds)
	A.Put(0, 4, 0.0)
	A.Put(0, 5, 0.0)

	A.Put(1, 0, 0.0)
	A.Put(1, 1, 1.0)
	A.Put(1, 2, 0.0)
	A.Put(1, 3, 0.0)
	A.Put(1, 4, deltaSeconds)
	A.Put(1, 5, 0.0)

	A.Put(2, 0, 0.0)
	A.Put(2, 1, 0.0)
	A.Put(2, 2, 1.0)
	A.Put(2, 3, 0.0)
	A.Put(2, 4, 0.0)
	A.Put(2, 5, deltaSeconds)

	A.Put(3, 0, 0.0)
	A.Put(3, 1, 0.0)
	A.Put(3, 2, 0.0)
	A.Put(3, 3, 1.0)
	A.Put(3, 4, 0.0)
	A.Put(3, 5, 0.0)

	A.Put(4, 0, 0.0)
	A.Put(4, 1, 0.0)
	A.Put(4, 2, 0.0)
	A.Put(4, 3, 0.0)
	A.Put(4, 4, 1.0)
	A.Put(4, 5, 0.0)

	A.Put(5, 0, 0.0)
	A.Put(5, 1, 0.0)
	A.Put(5, 2, 0.0)
	A.Put(5, 3, 0.0)
	A.Put(5, 4, 0.0)
	A.Put(5, 5, 1.0)
	return A
}

func (k *KalmanFilterGPSAccelerometer) GetLatLon() flightTerms.GeoPoint{
	return pilotMath.MetersToGeopoint(
		flightTerms.Meters(k.currentState.Get(1, 0)),
		flightTerms.Meters(k.currentState.Get(0, 0)),
	)
}

func NewKalmanFilterGPSAccelerometer(initialPoint flightTerms.GeoPoint, 
									 initialAltitude flightTerms.Meters,
								     latLonStandardDeviation flightTerms.Meters,
								     altitudeStandardDeviation flightTerms.Meters,
									 accelerometerNorthStandardDeviation float64,
								     accelerometerEastStandardDeviation float64,
								     accelerometerUpStandardDeviation float64,
								     currentTimestampSeconds float64) *KalmanFilterGPSAccelerometer{
	currentState := basicMatrix.NewMatrix(6, 1)

	currentState.Put(0, 0, float64(pilotMath.LongitudeToMeters(initialPoint.Longitude)))
	currentState.Put(1, 0, float64(pilotMath.LatitudeToMeters(initialPoint.Latitude)))
	currentState.Put(2, 0, float64(initialAltitude))
	currentState.Put(3, 0, 0.0)
	currentState.Put(4, 0, 0.0)
	currentState.Put(5, 0, 0.0)

	H := basicMatrix.NewIdentityMatrix(6, 6)
	P := basicMatrix.NewIdentityMatrix(6, 6)
	Q := basicMatrix.NewMatrix(6, 6)
	Q.Put(0, 0, accelerometerEastStandardDeviation * accelerometerEastStandardDeviation)
	Q.Put(1, 1, accelerometerNorthStandardDeviation * accelerometerNorthStandardDeviation)
	Q.Put(2, 2, accelerometerUpStandardDeviation * accelerometerUpStandardDeviation)

	R := basicMatrix.NewMatrix(6, 6)
	R.Put(0, 0, float64(latLonStandardDeviation * latLonStandardDeviation))
	R.Put(1, 1, float64(latLonStandardDeviation * latLonStandardDeviation))
	R.Put(2, 2, float64(altitudeStandardDeviation * altitudeStandardDeviation))
	// TODO might need to adjust these values...this is variance for velocity
	R.Put(3, 3, float64(latLonStandardDeviation * latLonStandardDeviation))
	R.Put(4, 4, float64(latLonStandardDeviation * latLonStandardDeviation))
	R.Put(4, 4, float64(altitudeStandardDeviation * altitudeStandardDeviation))
	return &KalmanFilterGPSAccelerometer{
		H: H,
		P: P,
		Q: Q,
		R: R,
		currentState: currentState,
		currentStateTimestampSeconds: currentTimestampSeconds,
	}
}

func main() {
	fmt.Printf("Start\n")
	var collection sensorDataCollection
	readFileAsJson("taco_bell_trip.json", &collection)
	initialSensorData := collection[0]
	kalmanFilter := NewKalmanFilterGPSAccelerometer(
		flightTerms.GeoPoint{
			Latitude: initialSensorData.GpsLat,
			Longitude: initialSensorData.GpsLon,
		},
		flightTerms.Meters(initialSensorData.GpsAlt),
		2.0, 
		3.5,
		0.05355371135598354,
		0.033436506994600976,
		0.208,
		initialSensorData.Timestamp,
	)
	startT := pilotMath.GetTimestampMS()
	for i := 1; i < len(collection); i++ {
		data := collection[i]

		kalmanFilter.Predict(
			float64(data.AbsNorthAcc),
			float64(data.AbsEastAcc),
			float64(data.AbsUpAcc),
			data.Timestamp,
		)

		if data.GpsLat != 0.0 {
			fmt.Printf("UPDATING with lat %f lon %f alt %f\n", data.GpsLat, data.GpsLon, data.GpsAlt)
			kalmanFilter.Update(
				data.GpsLat,
				data.GpsLon,
				data.GpsAlt,
			)
		}
		updatedPoint := kalmanFilter.GetLatLon()
		deltaT := data.Timestamp - initialSensorData.Timestamp 
		fmt.Printf("%f seconds in, Lat: %f, Lon: %f\n", deltaT, updatedPoint.Latitude, updatedPoint.Longitude)
	}
	fmt.Printf("got to end with no crash: %s\n", kalmanFilter)
	endT := pilotMath.GetTimestampMS()
	fmt.Printf("FINISH IN %f\n", (endT - startT))
}
