/*
 * PIDNumOutput.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: Evan
 */

#include <PIDNumOutput.h>

PIDNumOutput::PIDNumOutput() {
	value = 0.0;
}

void PIDNumOutput::PIDWrite(double output) {
	value = output;
}

double PIDNumOutput::getValue() {
	return value;
}

