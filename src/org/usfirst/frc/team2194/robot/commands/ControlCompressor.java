package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ControlCompressor extends Command {

	
	private static double holdTimeStart = 0;
	private static double nextHoldTime = 0;
	
    public ControlCompressor() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires (Robot.airCompressor);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
	RobotMap.compressor.setClosedLoopControl(true);
   	Robot.airCompressor.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.airCompressor.stop();
//    	
//    	if (Robot.holdCompressor && holdTimeStart == 0 && Timer.getFPGATimestamp()> nextHoldTime){
//    		holdTimeStart = Timer.getFPGATimestamp();
//    		Robot.airCompressor.stop();
//    	}
//    	if (holdTimeStart != 0 && (Timer.getFPGATimestamp() - holdTimeStart) > 5){
//    		Robot.holdCompressor = false;
//    		holdTimeStart = 0;
//    		Robot.airCompressor.start();
//    		nextHoldTime = Timer.getFPGATimestamp() + 10;
//    	}
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
    }
}
