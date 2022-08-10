#ifndef Delta_H
#define Delta_H

#include <Delta.h>

class Delta
{
	public:
		StepperMotor stepper_1(STEPPER_1_PIN_STEP,STEPPER_1_PIN_DIR);
		StepperMotor stepper_2(STEPPER_2_PIN_STEP,STEPPER_2_PIN_DIR);
		StepperMotor stepper_3(STEPPER_3_PIN_STEP,STEPPER_3_PIN_DIR);

		float stepAnglePrecision;

		float x_endEffectorcoordinate , y_endEffectorcoordinate , z_endEffectorcoordinate ;
		float x_endEffectorcoordinate_previous , y_endEffectorcoordinate_previous , z_endEffectorcoordinate_previous ;

		float jointAngle_1 , jointAngle_2 , jointAngle_3 ;
		float jointAngle_1_previous , jointAngle_2_previous , jointAngle_3_previous ;

		float arms_le , biceps_lf , endEffector_beta , fixedBase_alpha ;

		Delta(){

			stepAnglePrecision = 1.8;

			jointAngle_1=0;
			jointAngle_2=0;
			jointAngle_3=0;

			jointAngle_1_previous=0;
			jointAngle_2_previous=0;
			jointAngle_3_previous=0;

			arms_le=280;
			biceps_lf=100;
			endEffector_beta=50;
			fixedBase_alpha=100;

			angle2coordinates(0,0,0);

		}

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
				return();
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
				return();
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
				return();
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

		void angle2coordinates(float angle1, float angle2, float angle3) {

			jointAngle_1 = angle1 ;
			jointAngle_2 = angle2 ;
			jointAngle_3 = angle3 ;

			float ga = fixedBase_alpha-endEffector_beta;

			float V,W;
			float A_1,B_1,C_1,D_1;
			float A_2,B_2,C_2,D_2;
			float A_3,B_3,C_3,D_3;
			float P,Q,R,S,U,A,B,C;

			V=-2*biceps_lf*sin((PI/180)*(angle1)) ;
			W=arms_le*arms_le-biceps_lf*biceps_lf-fixedBase_alpha*fixedBase_alpha-endEffector_beta*endEffector_beta+2*fixedBase_alpha*endEffector_beta+2*endEffector_beta*biceps_lf*cos((PI/180)*(angle1))-2*fixedBase_alpha*biceps_lf*cos((PI/180)*(angle1)) ;

			A_1=sqrt(3)*(ga+biceps_lf*cos((PI/180)*(angle2))) ;
			B_1=(3*ga+2*biceps_lf*cos((PI/180)*(angle1))+biceps_lf*cos((PI/180)*(angle2))) ;
			C_1=2*biceps_lf*(sin((PI/180)*(angle2))-sin((PI/180)*(angle1))) ;
			D_1=-2*ga*biceps_lf*(cos((PI/180)*(angle1))-cos((PI/180)*(angle2))) ;

			A_2=sqrt(3)*(-ga-biceps_lf*cos((PI/180)*(angle3))) ;
			B_2=(3*ga+2*biceps_lf*cos((PI/180)*(angle1))+biceps_lf*cos((PI/180)*(angle3))) ;
			C_2=2*biceps_lf*(sin((PI/180)*(angle3))-sin((PI/180)*(angle1))) ;
			D_2=-2*ga*biceps_lf*(cos((PI/180)*(angle1))-cos((PI/180)*(angle3))) ;

			A_3=sqrt(3)*(-2*ga-biceps_lf*cos((PI/180)*(angle2))-biceps_lf*cos((PI/180)*(angle3))) ;
			B_3=biceps_lf*(cos((PI/180)*(angle3))-cos((PI/180)*(angle2))) ;
			C_3=2*biceps_lf*(sin((PI/180)*(angle3))-sin((PI/180)*(angle2))) ;
			D_3=-2*ga*biceps_lf*(cos((PI/180)*(angle2))-cos((PI/180)*(angle3))) ;

			V=-2*biceps_lf*sin((PI/180)*(angle1)) ;
			W=arms_le*arms_le-biceps_lf*biceps_lf-fixedBase_alpha*fixedBase_alpha-endEffector_beta*endEffector_beta+2*fixedBase_alpha*endEffector_beta+2*endEffector_beta*biceps_lf*cos((PI/180)*(angle1))-2*fixedBase_alpha*biceps_lf*cos((PI/180)*(angle1)) ;

			P=(D_1*A_2-D_2*A_1)/(B_1*A_2-B_2*A_1) ;
			Q=(C_2*A_1-C_1*A_2)/(B_1*A_2-B_2*A_1) ;
			R=(D_3-P*B_3)/A_3 ;
			S=-(C_3+Q*B_3)/A_3 ;
			U=2*(ga+biceps_lf*cos((PI/180)*(angle1))) ;

			A=1+S*S+Q*Q ;
			B=2*R*S+2*P*Q+U*Q+V ;
			C=R*R+P*P+U*P-W ;

			z_endEffectorcoordinate=(-B-sqrt(B*B-4*A*C))/(2*A) ;
			x_endEffectorcoordinate=R+S*z_endEffectorcoordinate ;
			y_endEffectorcoordinate=P+Q*z_endEffectorcoordinate ;

		}



		void moveStepperMotors(float angle1, float angle2, float angle3) {
			long step1 = long( angle1/stepAnglePrecision ) ;
			long step2 = long( angle2/stepAnglePrecision ) ;
			long step3 = long( angle3/stepAnglePrecision ) ;

			long loopSize = step1*step2*step3;

			int flag=0;

			for (long i = 0; i < loopSize; i++) {
				if ( i % (step2*step3) == 0 ) {
					stepper_1.highOnce();
					flag = 1;
				}
				if ( i % (step1*step3) == 0 ) {
					stepper_2.highOnce();
					flag = 1;
				}
				if ( i % (step2*step1) == 0 ) {
					stepper_3.highOnce();
					flag = 1;
				}

				if( flag == 1 ) {
					delayMicroseconds(stepper_1.m_stepDelay);
					stepper_1.lowOnce();
					stepper_2.lowOnce();
					stepper_3.lowOnce();
					delayMicroseconds(stepper_1.m_stepDelay);

					flag = 0;
				}

			}
		}


		void gotoXYZ(float x, float y, float z) {
			jointAngle_1_previous = jointAngle_1;
			jointAngle_2_previous = jointAngle_2;
			jointAngle_3_previous = jointAngle_3;

			x_endEffectorcoordinate_previous = x_endEffectorcoordinate;
			y_endEffectorcoordinate_previous = y_endEffectorcoordinate;
			z_endEffectorcoordinate_previous = z_endEffectorcoordinate;

			coordinates2angle(x,y,z);

			float angleDifferance1 = jointAngle_1 - jointAngle_1_previous ;
			float angleDifferance2 = jointAngle_2 - jointAngle_2_previous ;
			float angleDifferance3 = jointAngle_3 - jointAngle_3_previous ;

			moveStepperMotors(angleDifferance1, angleDifferance2, angleDifferance3);

		}


	private:



};
#endif //Delta_H
