#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <WPILib.h>

#include <Autonomous.h>

float Autonomous::distance = -1.0;
float Autonomous::movement = 0.0;
frc::Encoder* Autonomous::enc;
frc::RobotDrive* Autonomous::robotDrive;
frc::AnalogGyro* Autonomous::gyro;

void Autonomous::AutoInit(frc::Encoder* encoder, frc::RobotDrive* drive, frc::AnalogGyro* gyroscope) {
	enc = encoder;
	robotDrive = drive;
	gyro = gyroscope;

}

void Autonomous::baseGearRight(int autoState) {
	if (autoState == 0) {
		//Move forward until a distance is reached, across line
		if (enc->GetDistance() < 120.0) {
			robotDrive->MecanumDrive_Cartesian(0.0, -0.3, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 1;
			enc->Reset();
		}
	} else if (autoState == 1) {
		//Turn 45-ish degrees
		if (gyro->GetAngle() < 45.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.0,0.4);
		} else {
			robotDrive->StopMotor();
			autoState = 2;
			gyro->Reset();
		}
	} else if (autoState == 2) {
		//Line up with tape, while moving forward until close to gear

		if (distance > 20.0) {
			robotDrive->MecanumDrive_Cartesian(KP_MOVEMENT * movement, -0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 4;
		}
	}
}

	void Autonomous::forward(){
		if (enc->GetDistance() < 120.0) {
			robotDrive->MecanumDrive_Cartesian(0.0, -0.4, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			enc->Reset();
	}
