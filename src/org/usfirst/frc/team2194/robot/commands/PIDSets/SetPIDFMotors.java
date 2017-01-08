/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.commands.PIDSets;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2194.robot.Robot;

/**
 *
 * @author John
 */
public class SetPIDFMotors extends Command {

    public SetPIDFMotors() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        //requires(Robot.driveSystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {

        Robot.leftSideDrive.setPIDF(Robot.prefs.getDouble("Left Kp", .0003), Robot.prefs.getDouble("Left Ki",0),0,0);
        Robot.rightSideDrive.setPIDF(Robot.prefs.getDouble("Right Kp", .0003), Robot.prefs.getDouble("Right Ki",0),0,0);
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
        end();
    }
}
