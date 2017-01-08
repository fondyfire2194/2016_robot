package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PLSPortcullis extends Command {

	private double on1EncoderCounts;
	private double on2EncoderCounts;
	private double on3EncoderCounts;
	private double on4EncoderCounts;
	private double currentPosition;
	private boolean on1Dn;
	private boolean on2Dn;
	private boolean on3Dn;
	private double lift1StartTime;
	private double lift1Time;
	private boolean lift1OffDn;

	public PLSPortcullis() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		on1EncoderCounts = Robot.prefs.getDouble("Pcls ManDn", 3)
				* Robot.encoderCountsPerLoGearInch;
		on2EncoderCounts = Robot.prefs.getDouble("Pcls ManUp", 3)
				* Robot.encoderCountsPerLoGearInch;
		on3EncoderCounts = Robot.prefs.getDouble("Pcls Lift1 Up", 3)
				* Robot.encoderCountsPerLoGearInch;
		on4EncoderCounts = Robot.prefs.getDouble("Pcls End", 3)
				* Robot.encoderCountsPerLoGearInch;
		lift1Time = Robot.prefs.getDouble("Pcls Lift1 Time", 3);
		setTimeout(10);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		currentPosition = RobotMap.leftEncoder.getDistance();
		if (!on1Dn && (currentPosition > on1EncoderCounts)) {
			Robot.manipulators.portcullisChevalManipulatorLower();
			on1Dn = true;
		}
		if (!on2Dn && (currentPosition > on2EncoderCounts)) {
			Robot.manipulators.portcullisChevalManipulatorRaise();
			on2Dn = true;
		}
		if (!on3Dn && (currentPosition > on3EncoderCounts)) {
			Robot.manipulators.liftOneRaise();
			on3Dn = true;
			lift1StartTime = Timer.getFPGATimestamp();
		}
		if (!lift1OffDn && on3Dn && (Timer.getFPGATimestamp() - lift1StartTime) > lift1Time) {
			Robot.manipulators.liftOneSetOff();
			lift1OffDn = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut() ||currentPosition > on4EncoderCounts;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.manipulators.portcullisChevalManipulatorRaise();
		Robot.manipulators.liftOneLower();
		on1Dn = false;
		on2Dn = false;
		on3Dn = false;
		lift1OffDn = false;

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
