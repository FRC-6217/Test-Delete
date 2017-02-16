#include <PIDNumSource.h>
#include <WpiLib.h>

PIDNumSource::PIDNumSource(double num) {
	value = num;
}

void PIDNumSource::setInput(double num) {
	value = num;
}

double PIDNumSource::PIDGet() {
	return value;
}
