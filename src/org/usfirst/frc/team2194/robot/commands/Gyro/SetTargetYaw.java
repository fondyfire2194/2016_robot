/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.commands.Gyro;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2194.robot.Robot;

/**
 *
 * @author John
 */
public class SetTargetYaw extends Command {
    private double myTargetAngle;
    public SetTargetYaw(double targetAngle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	myTargetAngle = targetAngle;//Robot.gyroRotate.getGyroAngle();//targetAngle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
//    	myTargetAngle = Robot.gyroRotate.getGyroAngle();//targetAngle;
    	if (myTargetAngle <= 360 && myTargetAngle >= 180){
    		myTargetAngle = myTargetAngle - 360;
    	}
    	Robot.gyroRotate.targetAngle = myTargetAngle;
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
