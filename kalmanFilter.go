package main

import (
    "fmt"
    "encoding/json"
    "flightTerms"
    "pilotMath"
    "github.com/slobdell/basicMatrix"
    "io/ioutil"
)

const ACTUAL_GRAVITY = 9.80665


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

func (k *KalmanFilterGPSAccelerometer) Predict(absAccEast, timestampNow float64) {
    u := basicMatrix.NewMatrix(1, 1)

    u.Put(0, 0, absAccEast)
    deltaT := timestampNow - k.currentStateTimestampSeconds

    B := k.newControlMatrix(deltaT)

    A := k.newStateTransitionMatrix(deltaT)
    updatedState := (A.MultipliedBy(k.currentState)).Add(B.MultipliedBy(u))
    k.currentState = updatedState

    updatedPredictionMatrix := ((A.MultipliedBy(k.P)).MultipliedBy(A.Transpose())).Add(k.Q)
	// these are equivalent
    //updatedPredictionMatrix := (A.MultipliedBy(k.P.MultipliedBy(A.Transpose()))).Add(k.Q)
    k.P = updatedPredictionMatrix
	k.currentStateTimestampSeconds = timestampNow 
}

func (k *KalmanFilterGPSAccelerometer) Update(gpsLongitude, eastVelocity float64) {
    positionLongitude := float64(pilotMath.LongitudeToMeters(gpsLongitude))

    z := basicMatrix.NewMatrix(2, 1)
    z.Put(0, 0, positionLongitude)
	z.Put(1, 0, eastVelocity)

    y := z.Subtract(k.currentState)
    s := k.P.Add(k.R)
    sInverse, err := s.Inverse() 
    if err != nil {
        // matrix has no inverse, abort
        return
    }
    K := k.P.MultipliedBy(sInverse)

    updatedState := k.currentState.Add(K.MultipliedBy(y))
    k.currentState = updatedState

    updatedP := (basicMatrix.NewIdentityMatrix(2, 2).Subtract(K)).MultipliedBy(k.P)
	updatedP.PrettyPrint()
	fmt.Printf("\n")

	/*
	above is equivalent to:
		updatedP := k.P.Subtract(K.MultipliedBy(k.P))
	
	which would explain some confusion on the internets
	*/
    k.P = updatedP
}

func (k *KalmanFilterGPSAccelerometer) newControlMatrix(deltaSeconds float64) *basicMatrix.Matrix {
    B := basicMatrix.NewMatrix(2, 1)
    dtSquared := 0.5 * deltaSeconds * deltaSeconds

    B.Put(0, 0, dtSquared)
    B.Put(1, 0, deltaSeconds)
    return B
}
func (k *KalmanFilterGPSAccelerometer) newStateTransitionMatrix(deltaSeconds float64) *basicMatrix.Matrix {
    A := basicMatrix.NewMatrix(2, 2)
    A.Put(0, 0, 1.0)
    A.Put(0, 1, deltaSeconds)

    A.Put(1, 0, 0.0)
    A.Put(1, 1, 1.0)
    return A
}

func (k *KalmanFilterGPSAccelerometer) GetLon() float64 {
	point := pilotMath.MetersToGeopoint(
		0.0,
        flightTerms.Meters(k.currentState.Get(0, 0)),
    )
	return point.Longitude
}

func (k *KalmanFilterGPSAccelerometer) GetVelocityEast() float64 {
	return k.currentState.Get(1, 0)
}

func NewKalmanFilterGPSAccelerometer(initialLongitude float64, 
                				     latLonStandardDeviation flightTerms.Meters,
                				     accelerometerEastStandardDeviation float64,
                				     currentTimestampSeconds float64) *KalmanFilterGPSAccelerometer{
    currentState := basicMatrix.NewMatrix(2, 1)

    currentState.Put(0, 0, float64(pilotMath.LongitudeToMeters(initialLongitude)))
    currentState.Put(1, 0, 0.0)

    H := basicMatrix.NewIdentityMatrix(2, 2)
    P := basicMatrix.NewIdentityMatrix(2, 2)

    Q := basicMatrix.NewMatrix(2, 2)
    Q.Put(0, 0, accelerometerEastStandardDeviation * accelerometerEastStandardDeviation)
    Q.Put(1, 1, accelerometerEastStandardDeviation * accelerometerEastStandardDeviation)

    R := basicMatrix.NewMatrix(2, 2)
    R.Put(0, 0, float64(latLonStandardDeviation * latLonStandardDeviation))
    // TODO might need to play with this value
    R.Put(1, 1, float64(latLonStandardDeviation * latLonStandardDeviation))
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

	latLonStandardDeviation := flightTerms.Meters(2.0) // +/- 1m, increased for safety
	accelerometerEastStandardDeviation := ACTUAL_GRAVITY * 0.033436506994600976
    kalmanFilter := NewKalmanFilterGPSAccelerometer(
		initialSensorData.GpsLon,
        latLonStandardDeviation, 
		accelerometerEastStandardDeviation,
        initialSensorData.Timestamp,
    )
	previousLon := initialSensorData.GpsLon
    for i := 1; i < len(collection); i++ {
        data := collection[i]

		
		//fmt.Printf("Predicting with acc east: %f\n", float64(data.AbsEastAcc) * ACTUAL_GRAVITY)
        kalmanFilter.Predict(
            float64(data.AbsEastAcc) * ACTUAL_GRAVITY,
            data.Timestamp,
        )
		//dt := data.Timestamp - collection[i-1].Timestamp
		//fmt.Printf("I expect V to be this much higher than previous: %f\n", (float64(data.AbsEastAcc)) * dt)

        if data.GpsLat != 0.0 {
			eastVelocity := float64(
				pilotMath.LongitudeToMeters(data.GpsLon) - pilotMath.LongitudeToMeters(previousLon),
			)
            kalmanFilter.Update(
                data.GpsLon,
				eastVelocity,
            )
			previousLon = data.GpsLon

			predictedLon := kalmanFilter.GetLon()
			predictedV := kalmanFilter.GetVelocityEast()

			deltaT := data.Timestamp - initialSensorData.Timestamp 
			fmt.Printf("just updated with lon %f\n", data.GpsLon)
        }
		fmt.Printf("%f seconds in, Lon: %f, V(mph): %f, A: %f\n", deltaT, predictedLon, predictedV, float64(data.AbsEastAcc) * ACTUAL_GRAVITY)
    }
    fmt.Printf("got to end with no crash: %s\n", kalmanFilter)
}
