#include <WPILib.h>

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

const float KP_GYRO = -0.1;
const float KP_MOVEMENT = 0.01;
const float US_SCALE = 1/4.5;

class Autonomous {
public:
	static float movement;
	static float distance;

	static void AutoInit(frc::Encoder* encoder, frc::RobotDrive* drive, frc::AnalogGyro* gyroscope);
	static void baseGearRight(int autoState);
	static void baseGearCenter(int autoState);

private:
	static frc::Encoder* enc;
	static frc::RobotDrive* robotDrive;
	static frc::AnalogGyro* gyro;
};

#endif /* SRC_AUTONOMOUS_H_ */
