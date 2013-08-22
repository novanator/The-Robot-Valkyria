//Code 2. The written program in ROBOTC for the robot to move forward while maintaining balance in an upright position.

#pragma config(Sensor, S1,     gyroA,              sensorI2CHiTechnicGyro) 
#pragma config(Motor,  motorA,          motorA,             tmotorNXT, PIDControl, encoder) 
#pragma config(Motor,  motorB,          motorB,             tmotorNXT, PIDControl, encoder) 

task main(){ 

	int motorOutput = 0; 
	int gyroVelocity1 = 0; 
	int gyroVelocity2 = 0; 
	int gyroVelocity3 = 0; 
	int gyroAcceleration = 0; 
	int gyroAngle = 0; 
	int pidCalculation = 0; 
	int motorPosition = 0; 
	int velocityOffset = 0; 
	int forwardVelocity = 48; 

	const int GYRO_BIAS = 597; 
	const int FILTER_CONST = 90; 
	const float VELOCITY_CONST = 1.3; 
	const float ACCELERATION_CONST = 4.8; 
	const float ANGLE_CONST = 0.08; 
	 
	while(true){ 

		gyroVelocity1 = SensorValue(gyroA) - GYRO_BIAS; 
		gyroAcceleration = gyroVelocity1 - gyroVelocity3; 
		gyroAngle = gyroAngle + (gyroVelocity1 + gyroVelocity2 + gyroVelocity3)/3; 

		pidCalculation = gyroVelocity1*VELOCITY_CONST + gyroAcceleration*ACCELERATION_CONST 				   + gyroAngle*ANGLE_CONST; 

		motorOutput = ((100 - FILTER_CONST)*motorOutput+(pidCalculation*FILTER_CONST))/100; 

		motor[motorA] = (motorOutput) - forwardVelocity; 
		motor[motorB] = (motorOutput) - forwardVelocity; 

		wait1Msec(1); 

		gyroAngle = gyroAngle + motorOutput*0.13; 

		gyroVelocity3 = gyroVelocity2; 
		gyroVelocity2 = gyroVelocity1; 
	} 
}
