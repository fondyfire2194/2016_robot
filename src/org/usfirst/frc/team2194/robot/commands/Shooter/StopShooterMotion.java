package org.usfirst.frc.team2194.robot.commands.Shooter;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StopShooterMotion extends Command {
	private static double stopDistanceAngle;

	public StopShooterMotion() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.rightShooterMotor);
		requires(Robot.leftShooterMotor);
		requires(Robot.angleShooterMotor);
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		Robot.leftShooterMotor.valSet(0);
		Robot.rightShooterMotor.valSet(0);

		if (Robot.angleShooterMotor.getEncVelocity() > 0)
			stopDistanceAngle = 5;
		else
			stopDistanceAngle = -5;

		Robot.angleShooterMotor.valSet(Robot.angleShooterMotor.getPosition()
				+ stopDistanceAngle);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
