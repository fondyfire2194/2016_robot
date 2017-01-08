package org.usfirst.frc.team2194.robot.commands.Shooter;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RunLeftShooter extends Command {

	private double mySpeed;

	public RunLeftShooter(double speed) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftShooterMotor);

		mySpeed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		RobotMap.leftShooterMotor.changeControlMode(CANTalon.TalonControlMode.Speed);

		Robot.leftShooterMotor.enableControl();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (Robot.leftShooterMotor.isSpeedMode()) {
			Robot.leftShooterMotor.valSet(mySpeed);
			// System.out.println("SpeedMode");
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.leftShooterMotor.valSet(0);


	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
