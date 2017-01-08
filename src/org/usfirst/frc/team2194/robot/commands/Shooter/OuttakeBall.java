package org.usfirst.frc.team2194.robot.commands.Shooter;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class OuttakeBall extends Command {

    public OuttakeBall() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.leftShooterMotor);
    	requires(Robot.rightShooterMotor);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.leftShooterMotor.changeControlMode(CANTalon.TalonControlMode.Speed);
		Robot.leftShooterMotor.enableControl();
		RobotMap.rightShooterMotor.changeControlMode(CANTalon.TalonControlMode.Speed);
		Robot.rightShooterMotor.enableControl();
    	Robot.leftShooterMotor.valSet(-SmartDashboard.getNumber("Outtake Speed"));
    	Robot.rightShooterMotor.valSet(-SmartDashboard.getNumber("Outtake Speed"));
    	(new ShootBallSolenoid()).start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.leftShooterMotor.valSet(-SmartDashboard.getNumber("Outtake Speed"));
    	Robot.rightShooterMotor.valSet(-SmartDashboard.getNumber("Outtake Speed"));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.leftShooterMotor.valSet(0);
    	Robot.rightShooterMotor.valSet(0);
    	(new RetractBallSolenoid()).start();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
