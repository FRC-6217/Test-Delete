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
int Autonomous::autoState = 0;
frc::Encoder* Autonomous::enc;
frc::RobotDrive* Autonomous::robotDrive;
frc::AnalogGyro* Autonomous::gyro;

void Autonomous::AutoInit(frc::Encoder* encoder, frc::RobotDrive* drive, frc::AnalogGyro* gyroscope) {
	enc = encoder;
	robotDrive = drive;
	gyro = gyroscope;

}

void Autonomous::baseGearRight() {
	if (autoState == 0) {
		//Move forward until a distance is reached
		if (enc->GetDistance() < 100.0) {
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
	} else if (autoState == 3) {
		//Wait for ger to be removed
		robotDrive->StopMotor();
		if (/*todo: get gear state*/true) {
			autoState = 4;
		}
	} else if (autoState == 4) {
		if (enc->GetDistance() > -10.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 5;
			enc->Reset();
		}
	} else if (autoState == 5) {
		//turn to cross line
		if (gyro->GetAngle > -45.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.0,-0.4);
		} else {
			robotDrive->StopMotor();
			autoState = 6;
			gyro->Reset();
		}
	} else if (autoState == 6) {
		if (enc->GetDistance() < 30.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,-0.3, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 7;
			enc->Reset();
		}
	}
}

}

void Autonomous::baseGearleft() {
	if (autoState == 0) {
		//Move forward until a distance is reached
		if (enc->GetDistance() < 100.0) {
			robotDrive->MecanumDrive_Cartesian(0.0, -0.3, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 1;
			enc->Reset();
		}
	} else if (autoState == 1) {
		//Turn 45-ish degrees
		if (gyro->GetAngle() > -45.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.0,-0.4);
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
	} else if (autoState == 3) {
		//Wait for ger to be removed
		robotDrive->StopMotor();
		if (/*todo: get gear state*/true) {
			autoState = 4;
		}
	} else if (autoState == 4) {
		if (enc->GetDistance() > -10.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 5;
			enc->Reset();
		}
	} else if (autoState == 5) {
		//turn to cross line
		if (gyro->GetAngle < 45.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.0,0.4);
		} else {
			robotDrive->StopMotor();
			autoState = 6;
			gyro->Reset();
		}
	} else if (autoState == 6) {
		if (enc->GetDistance() < 30.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,-0.3, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 7;
			enc->Reset();
		}
	}
}

