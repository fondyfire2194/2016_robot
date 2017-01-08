package org.usfirst.frc.team2194.robot.commands.Lifter;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class JogUpLifter extends Command {

	public JogUpLifter() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		RobotMap.robotLiftMotor
				.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		RobotMap.robotLiftMotor.enable();
		Robot.airCompressor.stop();

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		RobotMap.robotLiftMotor.set(-SmartDashboard.getNumber("Lifter Speed"));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		RobotMap.robotLiftMotor.set(0);
		RobotMap.robotLiftMotor.disable();
		Robot.airCompressor.start();

		// Timer.delay(1);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

}
