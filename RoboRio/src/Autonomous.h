//!!! GARBAGE PROGAMMING ALERT !!!
//EVAN DOES NOT TAKE ANY RESPONABILITY FOR ANY MENTAL DAMAGES THAT MAY OCCUR

#include <WPILib.h>

//Creates definition of what's in the Autonomous class, and tells it to the main class.

//Standard c++ stuff. Only include if it hasn't been included already.
#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

//Some constants for autonomous.
const float KP_GYRO = -0.1;
const float KP_MOVEMENT = 0.01;
const float US_SCALE = 1/4.5;

//the class definition
class Autonomous {
public:
	//public member variables are set externally.
	static float movement;
	static float distance;
	static int autoState;

	//This is a list of the functions that are part of Autonomous.
	static void AutoInit(frc::Encoder* encoder, frc::RobotDrive* drive, frc::AnalogGyro* gyroscope, frc::DigitalInput* sw, frc::Spark* shoot, frc::Spark* revolve);
	static void baseGearCenter();
	static void forward();
	static void baseGearRight();
	static void baseGearLeft();
	static void ballShooter(int* next, bool team);
private:
	//private member variables are set internally.
	static frc::Encoder* enc;
	static frc::RobotDrive* robotDrive;
	static frc::AnalogGyro* gyro;
	static frc::DigitalInput* limitSwitch;
	static frc::Spark* shooter;
	static frc::Spark* revolver;

	static frc::Timer* timer;
};

#endif /* SRC_AUTONOMOUS_H_ */
