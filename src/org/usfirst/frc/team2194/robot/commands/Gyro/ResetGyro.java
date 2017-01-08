/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.commands.Gyro;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2194.robot.Robot;

/**
 *
 * @author John
 */
public class ResetGyro extends Command {
    
    public ResetGyro() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	setTimeout(.01);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.gyroRotate.resetGyro();
        
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		Robot.initialRollAngle = Robot.imu.getRoll();
		Robot.initialPitchAngle = Robot.imu.getPitch();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
