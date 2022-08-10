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
