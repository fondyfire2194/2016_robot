package org.usfirst.frc.team2194.robot.commands.Shooter;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShootPosition extends Command {

    public ShootPosition() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	(new PositionAngleShooter(120, .5,10)).start();
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
