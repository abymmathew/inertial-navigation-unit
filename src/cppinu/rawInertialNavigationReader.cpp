
#include "rawInertialNavigationReader.h"

RawInertialNavigationReader::RawInertialNavigationReader(int popFrequency, int sensorUpdateFrequency) {
	readCounter = 0;
	gpsIsReady = false;
    previousLat = NULL_SENTINEL_VALUE;
    previousLon = NULL_SENTINEL_VALUE;
    previousAlt = NULL_SENTINEL_VALUE;
    previousTowData = NULL_SENTINEL_VALUE;
	absoluteAccelerationNorth = 0.0;
	absoluteAccelerationEast = 0.0;
	absoluteAccelerationUp = 0.0;

	compass = new Compass3D(popFrequency);
	this->popFrequency = popFrequency;
	this->sensorUpdateFrequency = sensorUpdateFrequency;
	initGPS();
}

void RawInertialNavigationReader::initGPS() {
    if(!(gps.testConnection())) {
        printf("GPS CONNECTION UNSTABLE");
    } else {
        printf("Gps connection ok\n");
    }
    // This is a hack...without this the application crashes, I never took the time to figure out
    // root cause
    for (int i=0; i<100;i++) {
        gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data);
    }
}

void RawInertialNavigationReader::updateGPS() {
	gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data);
	towData = pos_data[0];
	altitude = pos_data[3] / 1000;
	latitude = pos_data[2] / 10000000;
	longitude = pos_data[1] / 10000000;
	if (abs(latitude) < 0.1 || abs(longitude) < 0.1) {
		gpsIsReady = false;
		return;
	}
	if (!(latitude == previousLat && longitude == previousLon && previousAlt && altitude && previousTowData == towData)) {
		gpsIsReady = true;
	}
	previousLat = latitude;
	previousLon = longitude;
	previousTowData = towData;
	previousAlt = altitude;
}

bool RawInertialNavigationReader::isGPSReady() {
	return gpsIsReady;
}

double RawInertialNavigationReader::getGPSAltitude() {
	gpsIsReady = false;
	return altitude;
}

double RawInertialNavigationReader::getLatitude(){
	gpsIsReady = false;
	return latitude;
}

double RawInertialNavigationReader::getLongitude(){
	gpsIsReady = false;
	return longitude;
}

void RawInertialNavigationReader::updateFromSensors(float magXOffset, float magYOffset, float magZOffset, float magneticDeclinationOffset) {
	int updatesPastSecond = readCounter % sensorUpdateFrequency;
	if(updatesPastSecond % (sensorUpdateFrequency / popFrequency) == 0) {
		updateGPS();
	}
	compass->updateFromSensor(magXOffset, magYOffset, magZOffset, magneticDeclinationOffset);
	readCounter++;
}

float RawInertialNavigationReader::getPitch(){
	return compass->getPitch();
}

float RawInertialNavigationReader::getYaw(float magneticDeclinationOffset){
	return compass->getYaw(magneticDeclinationOffset);
}
float RawInertialNavigationReader::getRoll(){
	return compass->getRoll();
}

float RawInertialNavigationReader::getRelativeForwardAcceleration(){
	return compass->getGroundForwardAcceleration();
}
float RawInertialNavigationReader::getRelativeUpAcceleration(){
	return compass->getGroundUpwardAcceleration();
}

void RawInertialNavigationReader::mutateAbsoluteAcceleration(float magXOffset, float magYOffset, float magZOffset, float magneticDeclinationOffset) {
	compass->startPopAbsoluteAcceleration(magXOffset, magYOffset, magZOffset, magneticDeclinationOffset);
	absoluteAccelerationNorth = compass->popAbsoluteAccelerationNorth();
	absoluteAccelerationEast = compass->popAbsoluteAccelerationEast();
	absoluteAccelerationUp = compass->popAbsoluteAccelerationUp();
}

float RawInertialNavigationReader::getAbsoluteAccelerationNorth(){
	return absoluteAccelerationNorth;
}

float RawInertialNavigationReader::getAbsoluteAccelerationEast(){
	return absoluteAccelerationEast;
}

float RawInertialNavigationReader::getAbsoluteAccelerationUp(){
	return absoluteAccelerationUp;
}
