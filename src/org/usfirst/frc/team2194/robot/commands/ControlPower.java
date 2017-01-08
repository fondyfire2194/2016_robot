package org.usfirst.frc.team2194.robot.commands;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ControlPower extends Command {
	private double channel0HighStartTime;
	private double channel1HighStartTime;
	private double channel14HighStartTime;
	private double channel15HighStartTime;

	public ControlPower() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.powerPanel);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.powerPanel.getTotalCurrent();
		if (Robot.powerPanel.getChannelCurrent(0) > 40
				&& channel0HighStartTime == 0) {
			channel0HighStartTime = Timer.getFPGATimestamp();
		}
		if (Robot.powerPanel.getChannelCurrent(1) > 40
				&& channel1HighStartTime == 0) {
			channel1HighStartTime = Timer.getFPGATimestamp();
		}
		if (Robot.powerPanel.getChannelCurrent(14) > 40
				&& channel14HighStartTime == 0) {
			channel14HighStartTime = Timer.getFPGATimestamp();
		}
		if (Robot.powerPanel.getChannelCurrent(15) > 40
				&& channel15HighStartTime == 0) {
			channel15HighStartTime = Timer.getFPGATimestamp();
		}
		if (Robot.powerPanel.getChannelCurrent(0) > 40
				&& Timer.getFPGATimestamp() > channel0HighStartTime + .5)
			Robot.channel0CurrentFault = true;
		if (Robot.powerPanel.getChannelCurrent(1) > 40
				&& Timer.getFPGATimestamp() > channel1HighStartTime + .5)
			Robot.channel1CurrentFault = true;
		if (Robot.powerPanel.getChannelCurrent(14) > 40
				&& Timer.getFPGATimestamp() > channel14HighStartTime + .5)
			Robot.channel14CurrentFault = true;
		if (Robot.powerPanel.getChannelCurrent(15) > 40
				&& Timer.getFPGATimestamp() > channel15HighStartTime + .5)
			Robot.channel15CurrentFault = true;
		if (Robot.powerPanel.getChannelCurrent(0) < 40)
			channel0HighStartTime = 0;
		if (Robot.powerPanel.getChannelCurrent(1) < 40)
			channel1HighStartTime = 0;
		if (Robot.powerPanel.getChannelCurrent(14) < 40)
			channel14HighStartTime = 0;
		if (Robot.powerPanel.getChannelCurrent(15) < 40)
			channel15HighStartTime = 0;

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
