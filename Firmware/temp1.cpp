void coordinates2angle(float x, float y, float z) {

	x_endEffectorcoordinate = x;
	y_endEffectorcoordinate = y;
	z_endEffectorcoordinate = z;

	float a1, b1, con1, y1a, y1b, z1a, z1b, a2, b2, con2, y2a, y2b, z2a, z2b, a3, b3, con3, y3a, y3b, z3a, z3b;
	float x_endEffectorcoordinate2, y_endEffectorcoordinate2, z_endEffectorcoordinate2;
	float x_endEffectorcoordinate3, y_endEffectorcoordinate3, z_endEffectorcoordinate3;


	a1 = (x_endEffectorcoordinate * x_endEffectorcoordinate + y_endEffectorcoordinate * y_endEffectorcoordinate + z_endEffectorcoordinate * z_endEffectorcoordinate + biceps_lf * biceps_lf - arms_le * arms_le - fixedBase_alpha * fixedBase_alpha + endEffector_beta * endEffector_beta - 2 * endEffector_beta * y_endEffectorcoordinate) / (2 * z_endEffectorcoordinate) ;
	b1 = (endEffector_beta - fixedBase_alpha - y_endEffectorcoordinate) / z_endEffectorcoordinate ;
	con1 = (fixedBase_alpha + a1 * b1) * (fixedBase_alpha + a1 * b1) - (1 + b1 * b1) * (fixedBase_alpha * fixedBase_alpha + a1 * a1 - biceps_lf * biceps_lf) ;
	if (con1 < 0) {
		Serial.print(" The chosen point is beyond the work space of the robot for the selected lengths of the parameters.\n") ;
	} else {
		y1a = (-(fixedBase_alpha + a1 * b1) + sqrt(con1)) / (1 + b1 * b1) ;
		y1b = (-(fixedBase_alpha + a1 * b1) - sqrt(con1)) / (1 + b1 * b1) ;
	}

	if (y1a < y1b) {
		z1a = a1 + b1 * y1a ;
		jointAngle_1 = (180 / PI) * (atan(z1a / abs(y1a + fixedBase_alpha))) ;
	} else {
		z1b = a1 + b1 * y1b ;
		jointAngle_1 = (180 / PI) * (atan(z1b / abs(y1b + fixedBase_alpha))) ;
		x_endEffectorcoordinate2 = (-x_endEffectorcoordinate + sqrt(3) * y_endEffectorcoordinate) / 2 ;
		y_endEffectorcoordinate2 = (-(sqrt(3) * x_endEffectorcoordinate + y_endEffectorcoordinate)) / 2 ;
		z_endEffectorcoordinate2 = z_endEffectorcoordinate ;
		a2 = (x_endEffectorcoordinate2 * x_endEffectorcoordinate2 + y_endEffectorcoordinate2 * y_endEffectorcoordinate2 + z_endEffectorcoordinate2 * z_endEffectorcoordinate2 + biceps_lf * biceps_lf - arms_le * arms_le - fixedBase_alpha * fixedBase_alpha + endEffector_beta * endEffector_beta - 2 * endEffector_beta * y_endEffectorcoordinate2) / (2 * z_endEffectorcoordinate2) ;
		b2 = (endEffector_beta - fixedBase_alpha - y_endEffectorcoordinate2) / z_endEffectorcoordinate2 ;
		con2 = (fixedBase_alpha + a2 * b2) * (fixedBase_alpha + a2 * b2) - (1 + b2 * b2) * (fixedBase_alpha * fixedBase_alpha + a2 * a2 - biceps_lf * biceps_lf) ;
	}

	if (con2 < 0) {
		Serial.print("The chosen point is beyond the work space of the robot for the selected lengths of the parameters.\n") ;
	} else {
		y2a = (-(fixedBase_alpha + a2 * b2) + sqrt(con2)) / (1 + b2 * b2) ;
		y2b = (-(fixedBase_alpha + a2 * b2) - sqrt(con2)) / (1 + b2 * b2) ;
	}

	if (y2a < y2b) {
		z2a = a2 + b2 * y2a ;
		jointAngle_2 = (180 / PI) * (atan(z2a / abs(y2a + fixedBase_alpha))) ;
	} else {
		z2b = a2 + b2 * y2b ;
		jointAngle_2 = (180 / PI) * (atan(z2b / abs(y2b + fixedBase_alpha))) ;
		x_endEffectorcoordinate3 = (-(x_endEffectorcoordinate + sqrt(3) * y_endEffectorcoordinate)) / 2 ;
		y_endEffectorcoordinate3 = (sqrt(3) * x_endEffectorcoordinate - y_endEffectorcoordinate) / 2 ;
		z_endEffectorcoordinate3 = z_endEffectorcoordinate ;
		a3 = (x_endEffectorcoordinate3 * x_endEffectorcoordinate3 + y_endEffectorcoordinate3 * y_endEffectorcoordinate3 + z_endEffectorcoordinate3 * z_endEffectorcoordinate3 + biceps_lf * biceps_lf - arms_le * arms_le - fixedBase_alpha * fixedBase_alpha + endEffector_beta * endEffector_beta - 2 * endEffector_beta * y_endEffectorcoordinate3) / (2 * z_endEffectorcoordinate3) ;
		b3 = (endEffector_beta - fixedBase_alpha - y_endEffectorcoordinate3) / z_endEffectorcoordinate3 ;
	}

	con3 = (fixedBase_alpha + a3 * b3) * (fixedBase_alpha + a3 * b3) - (1 + b3 * b3) * (fixedBase_alpha * fixedBase_alpha + a3 * a3 - biceps_lf * biceps_lf) ;

	if (con3 < 0) {
		Serial.print("The chosen point is beyond the work space of the robot for the selected lengths of the parameters.\n") ;
	} else {
		y3a = (-(fixedBase_alpha + a3 * b3) + sqrt(con3)) / (1 + b3 * b3) ;
		y3b = (-(fixedBase_alpha + a3 * b3) - sqrt(con3)) / (1 + b3 * b3) ;
	}

	if (y3a < y3b) {
		z3a = a3 + b3 * y3a ;
		jointAngle_3 = (180 / PI) * (atan(z3a / abs(y3a + fixedBase_alpha))) ;
	} else {
		z3b = a3 + b3 * y3b ;
		jointAngle_3 = (180 / PI) * (atan(z3b / abs(y3b + fixedBase_alpha))) ;
	}

}
