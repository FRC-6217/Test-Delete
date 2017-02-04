#ifndef drivetrain_H
#define drivetrain_H

#include "WpiLib.h"

#include <Commands/Subsystem.h>

class drivetrain : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	frc::RobotDrive* drive;

public:
	drivetrain();
	void InitDefaultCommand();
};

#endif  // drivetrain_H
