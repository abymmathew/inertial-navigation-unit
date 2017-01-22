
#ifndef RAW_INERTIAL_NAVIGATION_READER
#define RAW_INERTIAL_NAVIGATION_READER
#include "compass3d.h"
#include "Ublox.h"

#include <vector>
#include <iostream>

#define NULL_SENTINEL_VALUE -1000


class RawInertialNavigationReader
{
private:
	int popFrequency;
	int sensorUpdateFrequency;
	int readCounter;
	Compass3D *compass;
    std::vector<double> pos_data;
    Ublox gps;
	void initGPS();
	void updateGPS();

	bool gpsIsReady;
    double previousLat;
    double previousLon;
    double previousAlt;
    double previousTowData;

	double latitude;
	double longitude;
	double altitude;
	double towData;

	float absoluteAccelerationNorth;
	float absoluteAccelerationEast;
	float absoluteAccelerationUp;

public:
    RawInertialNavigationReader(int popFrequency, int sensorUpdateFrequency);

	bool isGPSReady();
    double getGPSAltitude();
    double getLatitude();
    double getLongitude();

    void updateFromSensors(float magXOffset, float magYOffset, float magZOffset, float magneticDeclinationOffset);
	void mutateAbsoluteAcceleration(float magXOffset, float magYOffset, float magZOffset, float magneticDeclinationOffset);

    float getPitch();
    float getYaw(float magneticDeclinationOffset);
    float getRoll();

	float getRelativeForwardAcceleration();
	float getRelativeUpAcceleration();

	float getAbsoluteAccelerationNorth();
	float getAbsoluteAccelerationEast();
	float getAbsoluteAccelerationUp();

};

#endif
