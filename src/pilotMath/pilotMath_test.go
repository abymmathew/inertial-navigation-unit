package pilotMath_test

import (
	"flightTerms"
	"math"
	"pilotMath"
	"testing"
	// "github.com/golang/mock/gomock"
)

func TestDegreesToRadians(t *testing.T) {
	expectedValue := flightTerms.Radians(1.5707963267948966)
	if pilotMath.DegreesToRadians(90) != expectedValue {
		t.Error("Degrees to radians failed")
	}
}

func TestRadiansToDegrees(t *testing.T) {
	expectedValue := flightTerms.Degrees(90.0)
	inputValue := flightTerms.Radians(1.5707963267948966)
	if pilotMath.RadiansToDegrees(inputValue) != expectedValue {
		t.Error("Radians to degrees failed")
	}
}

func TestPureNorth(t *testing.T) {
	fiveMPH := flightTerms.MetersPerSecond(2.2352)
	fakeVelocity := flightTerms.Velocity{
		LatitudeVelocity:  fiveMPH,
		LongitudeVelocity: flightTerms.MetersPerSecond(0.0),
		AltitudeVelocity:  fiveMPH,
	}

	expectedNorth := fiveMPH
	actualNorth := pilotMath.GetForwardMetersPerSecond(flightTerms.Degrees(0.0), fakeVelocity)
	if actualNorth != expectedNorth {
		t.Error("GetForwardMetersPerSecond North failed", expectedNorth, actualNorth)
	}

	expectedEast := flightTerms.MetersPerSecond(1e-15)
	actualEast := pilotMath.GetForwardMetersPerSecond(flightTerms.Degrees(90.00), fakeVelocity)
	if math.Abs(float64(actualEast)) > math.Abs(float64(expectedEast)) {
		t.Error("GetForwardMetersPerSecond East failed", expectedEast, actualEast)
	}
}

func TestNorthEast(t *testing.T) {
	fiveMPH := flightTerms.MetersPerSecond(2.2352)
	fakeVelocity := flightTerms.Velocity{
		LatitudeVelocity:  fiveMPH,
		LongitudeVelocity: fiveMPH,
		AltitudeVelocity:  fiveMPH,
	}
	expectedNorthEast := flightTerms.MetersPerSecond(3.1610501546163414)
	actualNorthEast := pilotMath.GetForwardMetersPerSecond(flightTerms.Degrees(45.0), fakeVelocity)
	if actualNorthEast != expectedNorthEast {
		t.Error("GetForwardMetersPerSecond NE failed", expectedNorthEast, actualNorthEast)
	}

	// current heading is 5 mph each way
	expectedNorthWest := flightTerms.MetersPerSecond(1e-15)
	actualNorthWest := pilotMath.GetForwardMetersPerSecond(flightTerms.Degrees(-45.0), fakeVelocity)
	if math.Abs(float64(actualNorthWest)) > math.Abs(float64(expectedNorthWest)) {
		t.Error("GetForwardMetersPerSecond NW failed", expectedNorthWest, actualNorthWest)
	}
}

func TestSouthEast(t *testing.T) {
	fiveMPH := flightTerms.MetersPerSecond(2.2352)
	fakeVelocity := flightTerms.Velocity{
		LatitudeVelocity:  -fiveMPH,
		LongitudeVelocity: fiveMPH,
		AltitudeVelocity:  fiveMPH,
	}

	expectedSouthEast := flightTerms.MetersPerSecond(3.1610501546163414)
	actualSouthEast := pilotMath.GetForwardMetersPerSecond(flightTerms.Degrees(135.0), fakeVelocity)
	if expectedSouthEast != expectedSouthEast {
		t.Error("GetForwardMetersPerSecond SE failed", expectedSouthEast, actualSouthEast)
	}

}

func TestSouthWest(t *testing.T) {
	fiveMPH := flightTerms.MetersPerSecond(2.2352)
	fakeVelocity := flightTerms.Velocity{
		LatitudeVelocity:  -fiveMPH,
		LongitudeVelocity: -fiveMPH,
		AltitudeVelocity:  fiveMPH,
	}

	expectedSouthWest := flightTerms.MetersPerSecond(3.1610501546163414)
	actualSouthWest := pilotMath.GetForwardMetersPerSecond(flightTerms.Degrees(-135.0), fakeVelocity)
	if expectedSouthWest != expectedSouthWest {
		t.Error("GetForwardMetersPerSecond SW failed", expectedSouthWest, actualSouthWest)
	}
}

func TestGetIntensity(t *testing.T) {
	expectedIntensity := flightTerms.Percent(0.5)
	actualIntensity := pilotMath.GetIntensity(0, -45, 90)
	if expectedIntensity != actualIntensity {
		t.Error("GetIntensity failed", expectedIntensity, actualIntensity)
	}
}

func TestGetIntensityWithinPercent(t *testing.T) {
	expectedIntensity := flightTerms.Percent(1.0)
	actualIntensity := pilotMath.GetIntensity(0, -180, 90)
	if expectedIntensity != actualIntensity {
		t.Error("GetIntensity failed", expectedIntensity, actualIntensity)
	}
}

func TestGetAngularVelocity(t *testing.T) {
	expectedVelocity := flightTerms.DegreesPerSecond(90.0)
	actualVelocity := pilotMath.GetAngularVelocity(0, -45, 0.5)
	if expectedVelocity != actualVelocity {
		t.Error("GetAngularVelocity failed", expectedVelocity, actualVelocity)
	}
}

func TestForceWithinRange(t *testing.T) {
	expectedValue := 0.0
	actualValue := pilotMath.ForceWithinRange(-1.0, 0.0, 1.0)
	if expectedValue != actualValue {
		t.Error("Values not equal", expectedValue, actualValue)
	}

	expectedValue = 1.0
	actualValue = pilotMath.ForceWithinRange(5.0, 0.0, 1.0)
	if expectedValue != actualValue {
		t.Error("Values not equal", expectedValue, actualValue)
	}

	expectedValue = 0.5
	actualValue = pilotMath.ForceWithinRange(0.5, 0.0, 1.0)
	if expectedValue != actualValue {
		t.Error("Values not equal", expectedValue, actualValue)
	}
}

func TestForceToPercent(t *testing.T) {
	expectedValue := flightTerms.Percent(1.0)
	actualValue := pilotMath.ForceToPercent(1.2)
	if expectedValue != actualValue {
		t.Error("Values not equal", expectedValue, actualValue)
	}
}

func TestCleanedAngle(t *testing.T) {
	expectedValue := flightTerms.Degrees(90.6)
	actualValue := pilotMath.CleanedAngle(flightTerms.Degrees(90.6))
	if expectedValue != actualValue {
		t.Error("Values not equal", expectedValue, actualValue)
	}

	expectedValue = flightTerms.Degrees(269.4)
	actualValue = pilotMath.CleanedAngle(flightTerms.Degrees(-90.6))
	if expectedValue != actualValue {
		t.Error("Values not equal", expectedValue, actualValue)
	}
}

func TestGetAzimuth(t *testing.T) {
	sanFrancisco := flightTerms.GeoPoint{
		Latitude:  37.7578885,
		Longitude: -122.5776859,
	}
	sanDiego := flightTerms.GeoPoint{
		Latitude:  32.8248175,
		Longitude: -117.375358,
	}
	seattle := flightTerms.GeoPoint{
		Latitude:  47.6149943,
		Longitude: -122.4759904,
	}
	newYork := flightTerms.GeoPoint{
		Latitude:  40.7058316,
		Longitude: -74.2581939,
	}
	d1 := pilotMath.GetAzimuth(sanFrancisco, sanDiego)

	expected := flightTerms.Degrees(137.7458328849339)
	if d1 != expected {
		t.Error("Values not equal", expected, d1)
	}
	d2 := pilotMath.GetAzimuth(seattle, newYork)
	expected = flightTerms.Degrees(83.28514087105475)
	if d2 != expected {
		t.Error("Values not equal", expected, d2)
	}

	d3 := pilotMath.GetAzimuth(newYork, seattle)
	expected = flightTerms.Degrees(297.97573161543903)
	if d3 != expected {
		t.Error("Values not equal", expected, d3)
	}
}

func TestGetDistance(t *testing.T) {
	sanFrancisco := flightTerms.GeoPoint{
		Latitude:  37.7578885,
		Longitude: -122.5776859,
	}
	sanDiego := flightTerms.GeoPoint{
		Latitude:  32.8248175,
		Longitude: -117.375358,
	}
	seattle := flightTerms.GeoPoint{
		Latitude:  47.6149943,
		Longitude: -122.4759904,
	}
	newYork := flightTerms.GeoPoint{
		Latitude:  40.7058316,
		Longitude: -74.2581939,
	}
	eastBay := flightTerms.GeoPoint{
		Latitude:  37.9487994,
		Longitude: -122.0508795,
	}
	d1 := pilotMath.GetDistance(sanFrancisco, sanDiego)

	expected := flightTerms.Meters(723484.8142990579)
	if d1 != expected {
		t.Error("Values not equal", expected, d1)
	}

	d2 := pilotMath.GetDistance(seattle, newYork)
	expected = flightTerms.Meters(3.857714269043667e+06)
	if d2 != expected {
		t.Error("Values not equal", expected, d2)
	}

	d3 := pilotMath.GetDistance(eastBay, sanFrancisco)
	expected = flightTerms.Meters(50891.21718090415)
	if d3 != expected {
		t.Error("Values not equal", expected, d3)
	}
}

func TestGetAngleDelta(t *testing.T) {
	expected := flightTerms.Degrees(-10)
	a1 := pilotMath.GetAngleDelta(
		flightTerms.Degrees(5),
		flightTerms.Degrees(355),
	)

	if a1 != expected {
		t.Error("Values not equal", expected, a1)
	}

	expected = 5
	a2 := pilotMath.GetAngleDelta(
		flightTerms.Degrees(180),
		flightTerms.Degrees(185),
	)
	if a2 != expected {
		t.Error("Values not equal", expected, a2)
	}

	expected = flightTerms.Degrees(10)
	a3 := pilotMath.GetAngleDelta(
		flightTerms.Degrees(355),
		flightTerms.Degrees(5),
	)
	if a3 != expected {
		t.Error("Values not equal", expected, a3)
	}
}

func TestGetPointAhead(t *testing.T) {
	expected := flightTerms.GeoPoint{
		Latitude:  37.77751249735129,
		Longitude: -122.30472645743545,
	}
	sanFrancisco := flightTerms.GeoPoint{
		Latitude:  37.7775672,
		Longitude: -122.4185077,
	}
	g1 := pilotMath.GetPointAhead(
		sanFrancisco,
		flightTerms.Meters(10000),
		flightTerms.Degrees(90),
	)
	if g1 != expected {
		t.Error("Values not equal", expected, g1)
	}

	expected = flightTerms.GeoPoint{
		Latitude:  37.77751249735129,
		Longitude: -122.5322889425646,
	}
	g2 := pilotMath.GetPointAhead(
		sanFrancisco,
		flightTerms.Meters(10000),
		flightTerms.Degrees(-90),
	)

	if g2 != expected {
		t.Error("Values not equal", expected, g2)
	}
	expected = flightTerms.GeoPoint{
		Latitude:  37.71394824460774,
		Longitude: -122.33812130872423,
	}
	g3 := pilotMath.GetPointAhead(
		sanFrancisco,
		flightTerms.Meters(10000),
		flightTerms.Degrees(135),
	)

	if g3 != expected {
		t.Error("Values not equal", expected, g3)
	}
}

func TestGetLeastCommonMultiple(t *testing.T) {
	actual := pilotMath.LeastCommonMultiple(4, 3)
	expected := int32(12)
	if actual != expected {
		t.Error("Values not equal", expected, actual)
	}
}

func TestFrequencyReached(t *testing.T) {
	actual := pilotMath.FrequencyReached(
		pilotMath.GetTimestampMS(),
		flightTerms.Hertz(math.Inf(1)),
	)
	expected := true
	if actual != expected {
		t.Error("Values not equal", expected, actual)
	}

	actual = pilotMath.FrequencyReached(
		pilotMath.GetTimestampMS(),
		flightTerms.Hertz(1),
	)
	expected = false
	if actual != expected {
		t.Error("Values not equal", expected, actual)
	}

	actual = pilotMath.FrequencyReached(
		pilotMath.GetTimestampMS()-0.2,
		flightTerms.Hertz(5),
	)
	expected = true
	if actual != expected {
		t.Error("Values not equal", expected, actual)
	}
}

func TestLeastCommonMultipleForMany(t *testing.T) {
	actual := pilotMath.LeastCommonMultipleForMany(2.0, 3.0, 5.0, 6.0, 7.0, 9.0)
	expected := int32(630)
	if actual != expected {
		t.Error("Values not equal", expected, actual)
	}
}

func TestGetCorrespondingAngularPercent(t *testing.T) {
	actual := pilotMath.GetCorrespondingAngularPercent(
		45.0,
		0.0,
		90.0,
	)
	expected := flightTerms.Percent(0.5)
	if actual != expected {
		t.Error("Values not equal", expected, actual)
	}

	actual = pilotMath.GetCorrespondingAngularPercent(
		45.0,
		20.0,
		90.0,
	)
	expected = flightTerms.Percent(0.35714285714285715)
	if actual != expected {
		t.Error("Values not equal", expected, actual)
	}
}
