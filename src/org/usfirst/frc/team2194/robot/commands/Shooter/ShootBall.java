package org.usfirst.frc.team2194.robot.commands.Shooter;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ShootBall extends Command {

	private double myStartTime;
	private boolean hitTime;

	public ShootBall() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftShooterMotor);
		requires(Robot.rightShooterMotor);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		hitTime = false;
		RobotMap.leftShooterMotor
				.changeControlMode(CANTalon.TalonControlMode.Speed);
		Robot.leftShooterMotor.enableControl();
		RobotMap.rightShooterMotor
				.changeControlMode(CANTalon.TalonControlMode.Speed);
		Robot.rightShooterMotor.enableControl();
		Robot.leftShooterMotor.valSet(SmartDashboard.getNumber("Shoot Speed"));
		Robot.rightShooterMotor.valSet(0);
		myStartTime = Timer.getFPGATimestamp();
		setTimeout(2);
//		Robot.holdCompressor = true;// inhibit compressor from starting
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (!Robot.inAutonomous || Robot.shootHighGoal) {
			if ((Timer.getFPGATimestamp() - myStartTime) > .25) {// delay right
																	// motor
																	// start
																	// for
																	// voltage
																	// surge
																	// reduction
																	// Robot.rightShooterMotor.valSet(SmartDashboard
				// .getNumber("Shoot Speed"));
				Robot.rightShooterMotor.valSet(Robot.angleShooterMotor
						.shooterSpeedCalculated());
			}
			// Robot.leftShooterMotor.valSet(SmartDashboard.getNumber("Shoot Speed"));
			Robot.leftShooterMotor.valSet(Robot.angleShooterMotor
					.shooterSpeedCalculated());
			if (hitTime == false
					&& (Timer.getFPGATimestamp() - myStartTime) > 1.5) {
//				if (Robot.inAutonomous && !Robot.visionRotate.targetOnScreen()){}
//				else{
				(new ShootBallSolenoid()).start();
				hitTime = true;
//				}
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut(); //|| (Robot.inAutonomous && !Robot.shootHighGoal);
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.leftShooterMotor.valSet(0);
		Robot.rightShooterMotor.valSet(0);
		Robot.holdCompressor = false;
		(new RetractBallSolenoid()).start();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
