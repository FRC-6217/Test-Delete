#ifndef MecanumDrive_H
#define MecanumDrive_H

#include "../CommandBase.h"

class MecanumDrive : public CommandBase {
public:
	MecanumDrive();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // MecanumDrive_H
