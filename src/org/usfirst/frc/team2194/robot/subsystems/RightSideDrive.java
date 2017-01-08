/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

/**
 *
 * @author John
 */
//
public class RightSideDrive extends PIDSubsystem {

	private static final double Kp = .000005; // was .000005
	private static final double Ki = .000005; // was .000001
	private static final double Kd = 0.0;
	private static final double Kf = 0.0;

	public Encoder encoder = RobotMap.rightEncoder;
	public SpeedController motor = RobotMap.driveRightMotor1;

	// Initialize your subsystem here
	public RightSideDrive() {
		super("RightSideDrive", Kp, Ki, Kd, Kf);
		getPIDController().disable();
		getPIDController().setSetpoint(0);
		getPIDController().setAbsoluteTolerance(5);
		getPIDController().setContinuous(false);

		// Use these to get going:
		// setSetpoint() - Sets where the PID controller should move the system
		// to
		// enable() - Enables the PID controller.
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		// setDefaultCommand(new DrivesSpeedSourceSelect());

	}

	protected double returnPIDInput() {
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
		return encoder.pidGet();
	}

	protected void usePIDOutput(double output) {
		RobotMap.driveRightMotor1
				.set(output
						+ (.5 * getPIDController().getSetpoint() / Robot.maxEncoderCountsPerSecond));
		// Use output to drive your system, like a motor
		// e.g. yourMotor.set(output);
	}

	public void setPIDF(double Kp, double Ki, double Kd, double Kf) {
		getPIDController().setPID(Kp, Ki, Kd, Kf);
	}

	public void zeroIGain() {
		getPIDController().setPID(getPIDController().getP(), 0,
				getPIDController().getD());
		return;
	}

	public void setSetpoint(double setpoint) {
		getPIDController().setSetpoint(setpoint);

	}

	public void setMaxOut(double speed) {
		getPIDController().setOutputRange(-speed, speed);
	}

	public void runRightMotor(double right) {
		// motor.set(right);
		RobotMap.driveRightMotor1.set(right);
	}

	public void updateStatus() {
		SmartDashboard.putNumber("Right Drive Motor Setting",
				-RobotMap.driveRightMotor1.get());
		SmartDashboard.putNumber("Right Encoder Position Inches",
				encoder.getDistance() / Robot.encoderCountsPerLoGearInch);
		SmartDashboard.putNumber("Right Ki", getPIDController().getI() * 1000);

		if (Robot.debugRightSpeedPID) {
			SmartDashboard.putBoolean("PID Right Enabled", getPIDController()
					.isEnabled());
			SmartDashboard.putNumber("PID Right Setpoint", getPIDController()
					.getSetpoint());
			SmartDashboard.putNumber("PID Right Output", getPIDController()
					.get());
			SmartDashboard.putNumber("PID Right Error", getPIDController()
					.getError());
			SmartDashboard.putBoolean("Right PID On Target", getPIDController()
					.onTarget());
			SmartDashboard.putBoolean("Right Stopped", encoder.getStopped());
			SmartDashboard.putNumber("Right Motor Speed RPM", encoder.getRate()
					* 60 / Robot.encoderCountsPerMotorRev);
			SmartDashboard.putNumber("Right Encoder Speed", encoder.getRate());
		}
	}
}
