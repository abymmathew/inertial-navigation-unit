package pilotMath

import (
    "flightTerms"
    "math"
    "time"
)

const EARTH_RADIUS = flightTerms.Meters(6371 * 1000.0)


func DegreesToRadians(degrees flightTerms.Degrees) flightTerms.Radians {
    return flightTerms.Radians(degrees * math.Pi / 180.0)
}

func RadiansToDegrees(radians flightTerms.Radians) flightTerms.Degrees {
    return flightTerms.Degrees(radians * 180.0 / math.Pi)
}


func GetForwardMetersPerSecond(azimuthRelativeToNorth flightTerms.Degrees, velocity flightTerms.Velocity) flightTerms.MetersPerSecond {
    // 0 degrees is pure Noth
    offsetDegrees := flightTerms.Degrees(90.0)
    invertedForCompass := azimuthRelativeToNorth * -1.0
    azimuthRadians := float64(DegreesToRadians(invertedForCompass) + DegreesToRadians(offsetDegrees))

    deltaY := float64(velocity.LatitudeVelocity)
    deltaX := float64(velocity.LongitudeVelocity)
    return flightTerms.MetersPerSecond(math.Cos(azimuthRadians) * deltaX + math.Sin(azimuthRadians) * deltaY)
}

func geoAngle(latOrLon float64) float64 {
    return float64(
        DegreesToRadians(
            flightTerms.Degrees(
                latOrLon,
            ),
        ),
    )
}


func GetAzimuth(fromCoordinate flightTerms.GeoPoint, toCoordinate flightTerms.GeoPoint) flightTerms.Degrees {

    deltaLongitude := geoAngle(toCoordinate.Longitude - fromCoordinate.Longitude)

    toLatRads := geoAngle(toCoordinate.Latitude)
    fromLatRads := geoAngle(fromCoordinate.Latitude)

    return CleanedAngle(
        RadiansToDegrees(
            flightTerms.Radians(
                math.Atan2(
                    math.Sin(deltaLongitude) * math.Cos(toLatRads),
                    (
                        math.Cos(fromLatRads) * math.Sin(toLatRads) -
                        math.Sin(fromLatRads) * math.Cos(toLatRads) * math.Cos(deltaLongitude)),
                ),
            ),
        ),
    )
}

func GetDistance(fromCoordinate flightTerms.GeoPoint, toCoordinate flightTerms.GeoPoint) flightTerms.Meters {
    deltaLon := geoAngle(toCoordinate.Longitude - fromCoordinate.Longitude)
    deltaLat := geoAngle(toCoordinate.Latitude - fromCoordinate.Latitude)

    a := math.Pow(math.Sin(deltaLat / 2.0), 2) +
         math.Cos(geoAngle(fromCoordinate.Latitude)) *
         math.Cos(geoAngle(toCoordinate.Latitude)) *
         math.Pow(math.Sin(deltaLon / 2.0), 2)
    c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1.0 - a))
    return EARTH_RADIUS * flightTerms.Meters(c)
}

func CleanedAngle(dirtyAngle flightTerms.Degrees) flightTerms.Degrees {
    // TODO feel free to refactor, test coverage exist.  mod arithmetic in Golang is whack
    for ;dirtyAngle < 0; {
        dirtyAngle += flightTerms.Degrees(360.0)
    }
    for ; dirtyAngle > 360; {
        dirtyAngle -= flightTerms.Degrees(360.0)
    }
    return dirtyAngle
}

func GetIntensity(target float64, current float64, maxPossible float64) flightTerms.Percent {
    return ForceToPercent(
        (target - current) / maxPossible,
    )
}

func GetAngularVelocity(latestAngle flightTerms.Degrees, earliestAngle flightTerms.Degrees, deltaSeconds flightTerms.Seconds) flightTerms.DegreesPerSecond {
    return flightTerms.DegreesPerSecond(
        float64(GetAngleDelta(earliestAngle, latestAngle)) / float64(deltaSeconds),
    )
}

func ForceWithinRange(value float64, minimum float64, maximum float64) float64 {
    if value < minimum {
        return minimum
    }
    if value > maximum {
        return maximum
    }
    return value
}

func ForceToPercent(dirtyPercent float64) flightTerms.Percent{
    return flightTerms.Percent(
        ForceWithinRange(
            dirtyPercent,
            -1.0,
            1.0,
        ),
    )
}

func MilesPerHourToMetersPerSecond(milesPerHour flightTerms.MilesPerHour) flightTerms.MetersPerSecond {
    return flightTerms.MetersPerSecond(milesPerHour * 0.44704)
}

func GetAngleDelta(fromAzimuth, toAzimuth flightTerms.Degrees) flightTerms.Degrees {
    deltaAzimuth := toAzimuth - fromAzimuth
    if deltaAzimuth > 180 {
        return deltaAzimuth - flightTerms.Degrees(360)
    }
    if deltaAzimuth < -180 {
        return deltaAzimuth + flightTerms.Degrees(360)
    }
    return deltaAzimuth
}

func GetPointAhead(fromCoordinate flightTerms.GeoPoint, distance flightTerms.Meters, azimuth flightTerms.Degrees) flightTerms.GeoPoint {
    radiusFraction := float64(distance / EARTH_RADIUS)

    bearing := float64(DegreesToRadians(azimuth))

    lat1 := geoAngle(fromCoordinate.Latitude)
    lng1 := geoAngle(fromCoordinate.Longitude)

    lat2_part1 := math.Sin(lat1) * math.Cos(radiusFraction)
    lat2_part2 := math.Cos(lat1) * math.Sin(radiusFraction) * math.Cos(bearing)

    lat2 := math.Asin(lat2_part1 + lat2_part2)

    lng2_part1 := math.Sin(bearing) * math.Sin(radiusFraction) * math.Cos(lat1)
    lng2_part2 := math.Cos(radiusFraction) - (math.Sin(lat1) * math.Sin(lat2))

    lng2 := lng1 + math.Atan2(lng2_part1, lng2_part2)
    lng2 = math.Mod((lng2 + 3 * math.Pi), (2 * math.Pi)) - math.Pi

    return flightTerms.GeoPoint {
        Latitude: float64(RadiansToDegrees(flightTerms.Radians(lat2))),
        Longitude: float64(RadiansToDegrees(flightTerms.Radians(lng2))),
    }
}

func PointPlusDistanceEast(fromCoordinate flightTerms.GeoPoint, distance flightTerms.Meters) flightTerms.GeoPoint {
	return GetPointAhead(fromCoordinate, distance, 90.0)
}

func PointPlusDistanceNorth(fromCoordinate flightTerms.GeoPoint, distance flightTerms.Meters) flightTerms.GeoPoint {
	return GetPointAhead(fromCoordinate, distance, 0.0)
}

func LatitudeToMeters(latitude float64) flightTerms.Meters {
	distance := GetDistance(
		flightTerms.GeoPoint{
			Latitude: latitude,
			Longitude: 0.0,
		},
		flightTerms.GeoPoint{
			Latitude: 0.0,
			Longitude: 0.0,
		},
	)
	if latitude < 0 {
		distance *= -1
	}
	return distance
}

func LongitudeToMeters(longitude float64) flightTerms.Meters {
	distance := GetDistance(
		flightTerms.GeoPoint{
			Latitude: 0.0,
			Longitude: longitude,
		},
		flightTerms.GeoPoint{
			Latitude: 0.0,
			Longitude: 0.0,
		},
	)
	if longitude < 0 {
		distance *= -1
	}
	return distance
}

func MetersToGeopoint(latAsMeters, lonAsMeters float64) flightTerms.GeoPoint {
	point := flightTerms.GeoPoint{}
	pointEast := PointPlusDistanceEast(point, lonAsMeters)
	pointNorthEast := PointPlusDistanceNorth(pointEast, latAsMeters)
	return pointNorthEast
}

func GetTimestamp() int32 {
    return int32(time.Now().Unix())
}

func GetTimestampMS() flightTerms.Timestamp {
    // TODO this function probably belongs in lib
    return flightTerms.Timestamp(time.Now().UnixNano() / int64(time.Millisecond)) / 1000.0
}

func GreatestCommonDivisor(n1, n2 int32) int32 {
    for i := int32(math.Min(float64(n1), float64(n2))); i >= int32(1); i-- {
        if n1 % i == 0 && n2 % i == 0 {
            return i
        }
    }
    return 1
}


func LeastCommonMultiple(n1, n2 int32) int32 {
    inf := int32(math.Inf(1))
    if n1 == inf || n2 == inf {
        return inf
    }
    return n1 / GreatestCommonDivisor(n1, n2) * n2;
}

func LeastCommonMultipleForMany(args ...float64) int32{
    runningLCM := int32(1)
    for _, v := range args {
        runningLCM = LeastCommonMultiple(runningLCM, int32(v))
    }
    return runningLCM
}

func FrequencyReached(lastActionTimestamp flightTerms.Timestamp, updatesPerSecond flightTerms.Hertz) bool {
    var period float64 = 1.0 / float64(updatesPerSecond)
    if GetTimestampMS() >= lastActionTimestamp + flightTerms.Timestamp(period) {
        return true
    }
    return false
}


func GetCorrespondingAngularPercent(inputAngle, minAngle, maxAngle flightTerms.Degrees) flightTerms.Percent {
    return flightTerms.Percent(
        (inputAngle - minAngle) / (maxAngle - minAngle),
    )
}
