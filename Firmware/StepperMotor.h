#ifndef StepperMotor_H
#define StepperMotor_H

#include <Arduino.h>

class StepperMotor
{
	public:
		const int m_stepPin,m_dirPin;
		byte m_enableStatus = 0;
		int m_stepDelay = 1000;

		StepperMotor(int stepPin, int dirPin);

		void step(byte dir, int numberOfSteps);
		void setStepDelay(int stepDelay);
		void enable();
		void disable();

		void highOnce(byte dir);
		void lowOnce();

	private:
};
#endif //StepperMotor_H
