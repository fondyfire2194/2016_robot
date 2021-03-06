package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.Shooter.PositionAngleShooter;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TravelConfig extends Command {

    public TravelConfig() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.angleShooterMotor);
    	requires(Robot.manipulators);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	(new PositionAngleShooter(37, .5,10)).start();
    	Robot.manipulators.liftOneLower();
		Robot.manipulators.liftTwoLower();
		Robot.manipulators.portcullisChevalManipulatorLower();
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
    }
}
