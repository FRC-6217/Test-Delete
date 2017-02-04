#include "drivetrain.h"
#include "../RobotMap.h"
#include "WpiLib.h"

drivetrain::drivetrain() : Subsystem("ExampleSubsystem") {
	drive = new frc::RobotDrive(FRONTLEFTMOTOR, BACKLEFTMOTOR, FRONTRIGHTMOTOR, BACKRIGHTMOTOR);
}

void drivetrain::InitDefaultCommand() {
	SetDefaultCommand(new MecanumDrive());
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
