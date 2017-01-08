package org.usfirst.frc.team2194.robot.commands.Shooter;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PositionAngleShooterToCamera extends Command {

	private double mySpeed;
	private double myYTarget;
	private double myYValue;
	private final double yValuePerDegree = 20; //was 16
	private double angleChangeDegrees;
	private double targetAngle;
	private double myTimeout;

	// position is in decimal degrees. 0 degrees is the shooter stowed position
	// angle counts up to max at shooter ball pickup position
	// speed is per unit setting - .5 is half speed
	// position is in Y value target from camera
	// Y value conversion to degrees
	// need to capture current angle and add / subtract Y value difference from
	// target value

	public PositionAngleShooterToCamera(double speed, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.angleShooterMotor);
		// myYTarget = yTarget;
		mySpeed = speed;
		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
//		(new SetPIDMotors(true)).start();
		Robot.usePID = true;
		
		myYTarget = Robot.visionRotate.getActiveYTarget();
		Robot.useGoalVision = true; //active rotate 
		RobotMap.angleShooterMotor
				.changeControlMode(CANTalon.TalonControlMode.Position);

		Robot.angleShooterMotor.enableControl();
		setTimeout(myTimeout);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		myYValue = Robot.yCameraVal;
		if (myYValue == 999 || myYValue == 1000)
			myYValue = myYTarget;
		angleChangeDegrees = (myYValue - myYTarget) / yValuePerDegree;
//		if (!Robot.reverseShoot)
			targetAngle = Robot.angleShooterMotor.getPosition()
					- (angleChangeDegrees / Robot.angleDegreesPerEncoderRev);
//		else
//			targetAngle = Robot.angleShooterMotor.getPosition()
//					+ angleChangeDegrees / Robot.angleDegreesPerEncoderRev;

		Robot.angleShooterMotor.setSpeedLimit(mySpeed * 12);
		Robot.angleShooterMotor.valSet(targetAngle);// position command
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return (Robot.visionRotate.onTarget() && Robot.inAutonomous);//(isTimedOut() || Robot.visionRotate.onTarget());// && Robot.inAutonomous;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.usePID = false;
//		(new SetPIDMotors(false)).start();
		Robot.useGoalVision = false; //stop rotate
		// Robot.angleShooterMotor.valSet(0);
		// RobotMap.angleShooterMotor.setPosition(RobotMap.angleShooterMotor
		// .getPosition());
		// RobotMap.angleShooterMotor
		// .changeControlMode(CANTalon.TalonControlMode.Position);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
