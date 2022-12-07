package org.team5148.chargedup;

import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;


import edu.wpi.first.wpilibj.drive.MecanumDrive;


public class Robot extends TimedRobot {

	//defining motor controlers 
	CANSparkMax frontLeft = new CANSparkMax(1, MotorType.kBrushless);
	CANSparkMax frontRight = new CANSparkMax(3, MotorType.kBrushless);
	CANSparkMax backLeft = new CANSparkMax(2, MotorType.kBrushless);
	CANSparkMax backRight = new CANSparkMax(4, MotorType.kBrushless);

	//xbox controller
	XboxController driveController = new XboxController(0);

	//define a drive
	MecanumDrive MDrive = new MecanumDrive(frontLeft , backLeft , frontRight , backRight);

	@Override
	public void autonomousInit() {

	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		frontRight.setInverted(true);
		backRight.setInverted(true);
		
	}

	@Override
	public void teleopPeriodic() {
		//use controller input to set drive system
		Double ySpeed = driveController.getLeftY();
		Double xSpeed = driveController.getLeftX();
		Double zRotation = driveController.getRightX(); 
		//math
		//angle-math.atan2
		//,math.2to
		//a^2+b^2=c^2

		double magnitude;
		double angle;
		
		MDrive.driveCartesian(ySpeed, xSpeed, zRotation);

		//Use this code
		
		MDrive.drivePolar(magnitude, angle, zRotation);

		
	}
}


