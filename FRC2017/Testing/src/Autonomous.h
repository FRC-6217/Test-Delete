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
	static int autoState;

	static void AutoInit(frc::Encoder* encoder, frc::RobotDrive* drive, frc::AnalogGyro* gyroscope, frc::DigitalInput* sw, frc::Spark* shoot, frc::Spark* revolve);
	static void baseGearCenter();
	static void forward();
	static void baseGearRight();
	static void baseGearLeft();
	static void ballShooter(int* next, bool team);
private:
	static frc::Encoder* enc;
	static frc::RobotDrive* robotDrive;
	static frc::AnalogGyro* gyro;
	static frc::DigitalInput* limitSwitch;
	static frc::Spark* shooter;
	static frc::Spark* revolver;

	static frc::Timer* timer;
};

#endif /* SRC_AUTONOMOUS_H_ */
