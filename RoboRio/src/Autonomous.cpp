//!!! GARBAGE PROGAMMING ALERT !!!
//EVAN DOES NOT TAKE ANY RESPONABILITY FOR ANY MENTAL DAMAGES THAT MAY OCCUR

//Libraries.
// Test

#include <iostream>
#include <memory>
#include <string> 

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <WPILib.h>

//Include the definition
#include <Autonomous.h>


//Declare all of the variables of the class
float Autonomous::distance = -1.0;
float Autonomous::movement = 0.0;
int Autonomous::autoState = 0;
frc::Encoder* Autonomous::enc;
frc::RobotDrive* Autonomous::robotDrive;
frc::AnalogGyro* Autonomous::gyro;
frc::DigitalInput* Autonomous::limitSwitch;
frc::Timer* Autonomous::timer;
frc::Spark* Autonomous::shooter;
frc::Spark* Autonomous::revolver;

//This function takes the motors and sensors, and allows Autonomous to reference them internally.
void Autonomous::AutoInit(frc::Encoder* encoder, frc::RobotDrive* drive, frc::AnalogGyro* gyroscope, frc::DigitalInput* sw, frc::Spark* shoot, frc::Spark* revolve) {
	enc = encoder;
	robotDrive = drive;
	gyro = gyroscope;
	limitSwitch = sw;
	shooter = shoot;
	revolver = revolve;

	timer = new frc::Timer();

}

//Auto modes. All use autoState, which gets incremented as we go.  Each autoState increment represents a stage of the program.
void Autonomous::baseGearRight() {
    revolver->Set(0.0);
    shooter->Set(0.0);
	frc::SmartDashboard::PutNumber("Auto state", autoState);
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
		//Move onto peg
		if (distance > 15.5) {
			robotDrive->MecanumDrive_Cartesian(0.0, -0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 4;
			timer->Reset();
			timer->Start();
		}
	} else if (autoState == 4) {
		//Wait for time period
		robotDrive->StopMotor();
		if (timer->Get() > 3.0) {
			autoState = 5;
			timer->Stop();
			timer->Reset();
		}
	} else if (autoState == 5) {
		//back up
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
		//Cross the line
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

//Just go forward.
void Autonomous::forward(){
    revolver->Set(0.0);
    shooter->Set(0.0);
	if (enc->GetDistance() < 120.0) {
		robotDrive->MecanumDrive_Cartesian(0.0, -0.4, KP_GYRO * gyro->GetAngle());
	} else {
		robotDrive->StopMotor();
	}
}

void Autonomous::baseGearLeft() {
    revolver->Set(0.0);
    shooter->Set(0.0);
	frc::SmartDashboard::PutNumber("Auto state", autoState);
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
			robotDrive->MecanumDrive_Cartesian(0.0,0.0,0.25);
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
		//Move onto peg
		if (distance > 15.5) {
			robotDrive->MecanumDrive_Cartesian(0.0, -0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 4;
			timer->Reset();
			timer->Start();
		}
	} else if (autoState == 4) {
		//Wait for timer
		robotDrive->StopMotor();
		if (timer->Get() > 3.0) {
			autoState = 4;
			timer->Stop();
			timer->Reset();
		}
	} else if (autoState == 5) {
		//back up
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
		//Cross line
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
    revolver->Set(0.0);
    shooter->Set(0.0);
	frc::SmartDashboard::PutNumber("Auto state", autoState);
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

		if (distance > 20.0) {
			robotDrive->MecanumDrive_Cartesian(movement, -0.15, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 3;
		}
	} else if (autoState == 3) {
		//Move onto peg
		if (distance > 15.5) {
			robotDrive->MecanumDrive_Cartesian(0.0, -0.2, KP_GYRO * gyro->GetAngle());
		} else {
			robotDrive->StopMotor();
			autoState = 4;
		}
	} else {
		robotDrive->StopMotor();
	}
}
//Oh boy.
void Autonomous::ballShooter(int* next, bool team) {
	if (autoState == 0) {
		//Stop and reset timer
		robotDrive->StopMotor();
		revolver->Set(0.0);
		shooter->Set(0.0);
		timer->Stop();
		timer->Reset();
		autoState = 1;
	} else if (autoState == 1) {
		//Spin up the shooter
		robotDrive->StopMotor();
		if (timer->Get() < 1.0) {
			revolver->Set(0.0);
			shooter->Set(0.5);
		} else {
			revolver->Set(0.0);
			shooter->Set(0.5);
			autoState = 2;
		}
	} else if (autoState == 2) {
		//Start the revolver, and continue for a time period
		robotDrive->StopMotor();
		if (timer->Get() < 5.0) {
			revolver->Set(-0.5);
			shooter->Set(0.5);
		} else {
			timer->Stop();
			timer->Reset();
			revolver->Set(0.0);
			shooter->Set(0.0);
			autoState = 3;
		}
	} else if (team && autoState == 3) {
		//if on the blue side, move away from the wall, as the robot is backwards.
		revolver->Set(0.0);
		shooter->Set(0.0);
		if (enc->Get() < 8) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.3,gyro->GetAngle() * KP_GYRO);
		} else {
			robotDrive->StopMotor();
			enc->Reset();
			autoState = 4;
		}
	} else if (team && autoState == 4) {
		//also if blue, turn 180 degrees
		revolver->Set(0.0);
		shooter->Set(0.0);
		if (gyro->GetAngle() < 180) {
			robotDrive->MecanumDrive_Cartesian(0.0,0.0,0.4);
		} else {
			robotDrive->StopMotor();
			gyro->Reset();
			enc->Reset();
			autoState = 5;
		}
	} else if (autoState == 5 || (autoState == 3 && !team)) {
		//After turning if necessary, reset everything, set autoState to 0, and tell the main class to call the correct gear method.
		revolver->Set(0.0);
		shooter->Set(0.0);
		robotDrive->StopMotor();
		autoState = 0;
		if (team) {
			*next = 1;
		} else {
			*next = 2;
		}
	}
}
