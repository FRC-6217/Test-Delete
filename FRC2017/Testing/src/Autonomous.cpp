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
frc::DigitalInput* Autonomous::limitSwitch;

void Autonomous::AutoInit(frc::Encoder* encoder, frc::RobotDrive* drive, frc::AnalogGyro* gyroscope, frc::DigitalInput* sw) {
	enc = encoder;
	robotDrive = drive;
	gyro = gyroscope;
	limitSwitch = sw;

}

void Autonomous::baseGearRight() {
	if (autoState == 0) {
		//Move forward until a distance is reached
		if (enc->GetDistance() < 75.0) {
			robotDrive->MecanumDrive_Cartesian(0.0, -0.3, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 1;
			enc->Reset();
		}
	} else if (autoState == 1) {
		//Turn 45-ish degrees
		if (gyro->GetAngle() > -45.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.0,-0.25);
		} else {
			robotDrive->StopMotor();
			autoState = 2;
			gyro->Reset();
			enc->Reset();
		}
	} else if (autoState == 2) {
		//Line up with tape, while moving forward until close to gear

		if (distance > 20.0) {
			robotDrive->MecanumDrive_Cartesian(movement, -0.15, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 3;
		}
	} else if (autoState == 3) {
		if (distance > 15.5) {
			robotDrive->MecanumDrive_Cartesian(0.0, -0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 4;
		}
	} else if (autoState == 4) {
		//Wait for gear to be removed
		robotDrive->StopMotor();
		if (limitSwitch->Get() == false) {
			autoState = 5;
		}
	} else if (autoState == 5) {
		if (enc->GetDistance() > -10.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 6;
			enc->Reset();
		}
	} else if (autoState == 6) {
		//turn to cross line
		if (gyro->GetAngle() < 45.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.0,0.4);
		} else {
			robotDrive->StopMotor();
			autoState = 7;
			gyro->Reset();
			enc->Reset();
		}
	} else if (autoState == 7) {
		if (enc->GetDistance() < 30.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,-0.3, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 8;
			enc->Reset();
		}
	} else {
		robotDrive->StopMotor();
	}
}

void Autonomous::forward(){
	if (enc->GetDistance() < 120.0) {
		robotDrive->MecanumDrive_Cartesian(0.0, -0.4, KP_GYRO * gyro->GetAngle());
	} else {
		robotDrive->StopMotor();
	}
}

void Autonomous::baseGearLeft() {
	if (autoState == 0) {
		//Move forward until a distance is reached
		if (enc->GetDistance() < 79.0) {
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
			enc->Reset();
		}
	} else if (autoState == 2) {
		//Line up with tape, while moving forward until close to gear

		if (distance > 25.0) {
			robotDrive->MecanumDrive_Cartesian(movement, -0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 3;
		}
	} else if (autoState == 3) {
		if (distance > 17.0) {
			robotDrive->MecanumDrive_Cartesian(0.0, -0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 4;
		}
	} else if (autoState == 4) {
		//Wait for ger to be removed
		robotDrive->StopMotor();
		if (limitSwitch->Get() == false) {
			autoState = 4;
		}
	} else if (autoState == 5) {
		if (enc->GetDistance() > -10.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 6;
			enc->Reset();
		}
	} else if (autoState == 6) {
		//turn to cross line
		if (gyro->GetAngle() > -45.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.0,-0.4);
		} else {
			robotDrive->StopMotor();
			autoState = 7;
			gyro->Reset();
			enc->Reset();
		}
	} else if (autoState == 7) {
		if (enc->GetDistance() < 30.0) {
			robotDrive->MecanumDrive_Cartesian(0.0,-0.3, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 8;
			enc->Reset();
		}
	} else {
		robotDrive->StopMotor();
	}
}
void Autonomous::baseGearCenter() {
	if (autoState == 0) {
		//Move forward until a distance is reached, across line
		if (enc->GetDistance() < 60.0) {
			robotDrive->MecanumDrive_Cartesian(0.0, -0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 1;
			enc->Reset();
		}

	} else if (autoState == 1) {
		//Line up with tape, while moving forward until close to gear

		if (distance > 25.0) {
			robotDrive->MecanumDrive_Cartesian(movement, -0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 3;
		}
	} else if (autoState == 3) {
		if (distance > 17.0) {
			robotDrive->MecanumDrive_Cartesian(0.0, -0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 4;
		}
	} else {
		robotDrive->StopMotor();
	}
}
