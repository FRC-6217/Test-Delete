#ifndef SRC_PIDNUMOUTPUT_H_
#define SRC_PIDNUMOUTPUT_H_

#include <PIDOutput.h>

class PIDNumOutput : public frc::PIDOutput {
public:
	PIDNumOutput();

	void PIDWrite(double output);
	double getValue();
private:
	double value;
};

#endif /* SRC_PIDNUMOUTPUT_H_ */
