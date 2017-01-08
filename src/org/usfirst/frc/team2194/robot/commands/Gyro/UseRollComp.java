package org.usfirst.frc.team2194.robot.commands.Gyro;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class UseRollComp extends Command {
private boolean mySet;
    public UseRollComp(boolean set) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	mySet = set;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.useRollComp = mySet;;
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
