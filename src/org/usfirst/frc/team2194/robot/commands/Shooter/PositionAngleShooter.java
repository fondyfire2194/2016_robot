package org.usfirst.frc.team2194.robot.commands.Shooter;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PositionAngleShooter extends Command {

	private double mySpeed;
	private double myPosition;
	private double timeStarted;
	private double myTimeout;

	// position is in decimal degrees. 0 degrees is the shooter stowed position
	// angle counts up to max at shooter ball pickup position
	// speed is per unit setting - .5 is half speed

	public PositionAngleShooter(double position, double speed, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.angleShooterMotor);
		myPosition = position;
		mySpeed = speed;
		myTimeout = timeout;
		// speed scale factor is gear ratio /360
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		RobotMap.angleShooterMotor
				.changeControlMode(CANTalon.TalonControlMode.Position);

		Robot.angleShooterMotor.enableControl();
		timeStarted = Timer.getFPGATimestamp();
		setTimeout(myTimeout);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.angleShooterMotor.setSpeedLimit(mySpeed * 12);
		Robot.angleShooterMotor.valSet(myPosition
				/ Robot.angleDegreesPerEncoderRev);// position command in
													// degrees convert to
													// encoder revs
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// return RobotMap.angleShooterMotor.getSpeed() == 0 &&
		// Timer.getFPGATimestamp() > (timeStarted + 1);
		return isTimedOut() || (Math
				.abs((Robot.angleShooterMotor.getPosition() * Robot.angleDegreesPerEncoderRev)
						- myPosition) < 2)
				&& Timer.getFPGATimestamp() > (timeStarted + 1);
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
