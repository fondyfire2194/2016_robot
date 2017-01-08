package org.usfirst.frc.team2194.robot.commands.RobotMotion;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RobotCrossPortcullis extends Command {
	private double mySpeed;
	private double inPositionBandEncoderCounts;
	private boolean myDisableWhenDone;
	private double myTimeout;
	private double rampIncrement;
	private double distanceEncoderCounts;
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

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in inches

	public RobotCrossPortcullis(double distance, double speed,
			double inPositionBand, boolean disableWhenDone, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leftSideDrive);
		requires(Robot.rightSideDrive);

		distanceEncoderCounts = distance * Robot.encoderCountsPerLoGearInch;
		mySpeed = speed;
		inPositionBandEncoderCounts = inPositionBand
				* Robot.encoderCountsPerLoGearInch;
		myDisableWhenDone = disableWhenDone;
		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.leftSideDrive.zeroIGain();
		Robot.rightSideDrive.zeroIGain();
		on1EncoderCounts = Robot.prefs.getDouble("Pcls ManDn", 3)
				* Robot.encoderCountsPerLoGearInch;
		on2EncoderCounts = Robot.prefs.getDouble("Pcls ManUp", 3)
				* Robot.encoderCountsPerLoGearInch;
		on3EncoderCounts = Robot.prefs.getDouble("Pcls Lift1 Up", 3)
				* Robot.encoderCountsPerLoGearInch;
		on4EncoderCounts = Robot.prefs.getDouble("Pcls End", 3)
				* Robot.encoderCountsPerLoGearInch;
		lift1Time = Robot.prefs.getDouble("Pcls Lift1 Time", 3);

		rampIncrement = mySpeed / 50;
		Robot.currentMaxSpeed = 0;
		Robot.leftPositionLinear.setSetpoint(distanceEncoderCounts);
		Robot.rightPositionLinear.setSetpoint(distanceEncoderCounts);
		Robot.leftPositionLinear.enable();
		Robot.rightPositionLinear.enable();
		Robot.isPositioning = true;
		Robot.isSamePositioning = true;
		setTimeout(myTimeout);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.currentMaxSpeed = Robot.currentMaxSpeed + rampIncrement;
		if (Robot.currentMaxSpeed > mySpeed)
			Robot.currentMaxSpeed = mySpeed;
		Robot.leftPositionLinear.setMaxOut(Robot.currentMaxSpeed);
		Robot.rightPositionLinear.setMaxOut(Robot.currentMaxSpeed);
		Robot.rightSideDrive.setMaxOut(Robot.currentMaxSpeed);
		Robot.leftSideDrive.setMaxOut(Robot.currentMaxSpeed);
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
		if (!lift1OffDn && on3Dn
				&& (Timer.getFPGATimestamp() - lift1StartTime) > lift1Time) {
			Robot.manipulators.liftOneSetOff();
			lift1OffDn = true;
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut()
				|| (Robot.leftPositionLinear
						.inPosition(inPositionBandEncoderCounts) && Robot.rightPositionLinear
						.inPosition(inPositionBandEncoderCounts));
	}

	// Called once after isFinished returns true
	protected void end() {
		if (myDisableWhenDone) {
			Robot.leftPositionLinear.disablePID();
			Robot.rightPositionLinear.disablePID();
		}
		Robot.isPositioning = false;
		Robot.isSamePositioning = false;
		Robot.leftPositionLinear.setMaxOut(1); // Ethan added this block of code
		Robot.rightPositionLinear.setMaxOut(1);
		Robot.rightSideDrive.setMaxOut(1);
		Robot.leftSideDrive.setMaxOut(1);
		Robot.manipulators.portcullisChevalManipulatorRaise();
		Robot.manipulators.liftOneLower();
		on1Dn = false;
		on2Dn = false;
		on3Dn = false;
		lift1OffDn = false;

		Robot.leftSideDrive.setPIDF(Robot.prefs.getDouble("Left Kp", .0003),
				Robot.prefs.getDouble("Left Ki", 0), 0, 0);
		Robot.rightSideDrive.setPIDF(Robot.prefs.getDouble("Right Kp", .0003),
				Robot.prefs.getDouble("Right Ki", 0), 0, 0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
