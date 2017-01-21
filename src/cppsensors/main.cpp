#include "rawInertialNavigationReader.h"
#define POP_FREQUENCY 16
#define ACTUAL_ACCELEROMETER_UPDATE_FREQUENCY 1600

float xOffset = -43.6189453125;
float yOffset = 13.7592773437;
float zOffset = -9.2390625;

float magneticDeclinationOffset = 13.8;
int counter = 0;


int main() {
	RawInertialNavigationReader *reader = new RawInertialNavigationReader(POP_FREQUENCY, ACTUAL_ACCELEROMETER_UPDATE_FREQUENCY);
	while(1) {
		reader->updateFromSensors(xOffset, yOffset, zOffset, magneticDeclinationOffset);
		if(counter % (ACTUAL_ACCELEROMETER_UPDATE_FREQUENCY / POP_FREQUENCY) == 0) {
			reader->mutateAbsoluteAcceleration(xOffset, yOffset, zOffset, magneticDeclinationOffset);

			float pitch = reader->getPitch();
			float yaw = reader->getYaw(magneticDeclinationOffset);
			float roll = reader->getRoll();
			float relForwardAcc = reader->getRelativeForwardAcceleration();
			float relUpAcc = reader->getRelativeUpAcceleration();

			float absNorthAcc = reader->getAbsoluteAccelerationNorth();
			float absEastAcc = reader->getAbsoluteAccelerationEast();
			float absUpAcc = reader->getAbsoluteAccelerationUp();
			float gpsLat = 0.0;
			float gpsLon = 0.0;
			float gpsAlt = 0.0;
			if(reader->isGPSReady()) {
				gpsLat = reader->getLatitude();
				gpsLon = reader->getLongitude();
				gpsAlt = reader->getGPSAltitude();
				printf("gps alt from main: %f", gpsAlt);
			}
			printf("P: %.1f, Y: %.1f, R: %.1f, ForwardA: %.3f, UpA: %.3f, abs N: %.3f, abs E: %.3f, abs Up: %.3f, lat: %f, lon: %f, alt: %f\n", pitch, yaw, roll, relForwardAcc, relUpAcc, absNorthAcc, absEastAcc, absUpAcc, gpsLat, gpsLon, gpsAlt);
		}
		counter++;
	}
	return 0;
}
