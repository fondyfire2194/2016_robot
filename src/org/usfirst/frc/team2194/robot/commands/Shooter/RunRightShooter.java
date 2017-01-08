package org.usfirst.frc.team2194.robot.commands.Shooter;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunRightShooter extends Command {
	private double mySpeed;

	public RunRightShooter(double speed) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.rightShooterMotor);
		mySpeed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		RobotMap.rightShooterMotor.changeControlMode(CANTalon.TalonControlMode.Speed);
		Robot.rightShooterMotor.enableControl();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (Robot.rightShooterMotor.isSpeedMode()) {
			Robot.rightShooterMotor.valSet(mySpeed);
			// System.out.println("SpeedMode");
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.rightShooterMotor.valSet(0);


	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
