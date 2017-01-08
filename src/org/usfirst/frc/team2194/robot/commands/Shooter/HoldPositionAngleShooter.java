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
public class HoldPositionAngleShooter extends Command {

	// holds arm in the position it was at when this command was called

	private double startPosition;
	private double myStartTime;
	private boolean hitTime1;
	private boolean hitTime2;

	public HoldPositionAngleShooter() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.angleShooterMotor);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// myStartTime = Timer.getFPGATimestamp();
		// hitTime1 = false;
		// hitTime2 = false;
		// Timer.delay(.5);
		startPosition = RobotMap.angleShooterMotor.getPosition();// encoder revs
		// RobotMap.angleShooterMotor.set(startPosition);
		// SmartDashboard.putNumber("Start Angle Found", startPosition);
		RobotMap.angleShooterMotor.changeControlMode(CANTalon.TalonControlMode.Position);
		// Robot.angleShooterMotor.valSet(startPosition);//encoder revs
		// Timer.delay(.5);

		Robot.angleShooterMotor.enableControl();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		// if ((Timer.getFPGATimestamp() - myStartTime) > 1) {//delay right
		// motor start for voltage surge reduction
		// Robot.rightShooterMotor.valSet(SmartDashboard
		// .getNumber("Shoot Speed"));
		// }

		Robot.angleShooterMotor.valSet(startPosition);
//		if (RobotMap.angleShooterMotor.getPosition() < (Robot.shooterAngleHomePosition + 3)) //Ethan commented these 4 lines out
//			RobotMap.angleShooterMotor.setProfile(1);
//		else
//			RobotMap.angleShooterMotor.setProfile(0);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
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
