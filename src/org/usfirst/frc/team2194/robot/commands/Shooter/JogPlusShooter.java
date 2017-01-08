package org.usfirst.frc.team2194.robot.commands.Shooter;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class JogPlusShooter extends Command {
	private double mySpeed;
    public JogPlusShooter(double speed) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	mySpeed = speed;
    	requires (Robot.angleShooterMotor);
    }

    // Called just before this Command runs the first time
	protected void initialize() {
		RobotMap.angleShooterMotor
				.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.angleShooterMotor.valSet(mySpeed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.angleShooterMotor.valSet(0);
//		Timer.delay(1);
//		RobotMap.angleShooterMotor.setPosition(RobotMap.angleShooterMotor
//				.getPosition());
		RobotMap.angleShooterMotor
				.changeControlMode(CANTalon.TalonControlMode.Position);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

}
