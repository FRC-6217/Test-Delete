#ifndef SRC_PIDNUMSOURCE_H_
#define SRC_PIDNUMSOURCE_H_

#include <PIDSource.h>

class PIDNumSource : public frc::PIDSource {
public:
	PIDNumSource(double num);

	void setInput(double num);
	double PIDGet();

private:
	double value;
};

#endif /* SRC_PIDNUMSOURCE_H_ */
