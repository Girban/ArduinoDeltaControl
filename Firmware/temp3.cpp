void moveStepperMotors(float angle1, float angle2, float angle3) {
	long step1 = long( angle1/stepAnglePrecision ) ;
	long step2 = long( angle2/stepAnglePrecision ) ;
	long step3 = long( angle3/stepAnglePrecision ) ;

	long loopSize = step1*step2*step3;
	long a=0, b=0, c=0;

	Serial.print("\n Step 1 , Step 2 , Step 3 : ");
	Serial.print(step1);
	Serial.print(" , ");
	Serial.print(step2);
	Serial.print(" , ");
	Serial.print(step3);
	Serial.println();

	for (long i = 0; i < loopSize; i++) {
		if ( i % (step2*step3) == 0 ) {
			a++;
			Serial.print("\n 1 , 2 , 3 : ");
			Serial.print(step1);
			Serial.print(" , ");
			Serial.print(step2);
			Serial.print(" , ");
			Serial.print(step3);
			Serial.print("\n a , b , c : ");
			Serial.print(a);
			Serial.print(" , ");
			Serial.print(b);
			Serial.print(" , ");
			Serial.print(c);
			Serial.println();

		}
		if ( i % (step1*step3) == 0 ) {
			b++;
			Serial.print("\n 1 , 2 , 3 : ");
			Serial.print(step1);
			Serial.print(" , ");
			Serial.print(step2);
			Serial.print(" , ");
			Serial.print(step3);
			Serial.print("\n a , b , c : ");
			Serial.print(a);
			Serial.print(" , ");
			Serial.print(b);
			Serial.print(" , ");
			Serial.print(c);
			Serial.println();
		}
		if ( i % (step2*step1) == 0 ) {
			c++;
			Serial.print("\n 1 , 2 , 3 : ");
			Serial.print(step1);
			Serial.print(" , ");
			Serial.print(step2);
			Serial.print(" , ");
			Serial.print(step3);
			Serial.print("\n a , b , c : ");
			Serial.print(a);
			Serial.print(" , ");
			Serial.print(b);
			Serial.print(" , ");
			Serial.print(c);
			Serial.println();
		}

	}
}
