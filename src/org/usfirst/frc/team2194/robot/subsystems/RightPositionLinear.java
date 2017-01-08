/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

/**
 *
 * @author John
 */
public class RightPositionLinear extends PIDSubsystem {

	private static final double Kp = .005;
	private static final double Ki = 0.0;
	private static final double Kd = 0.0;

	// Initialize your subsystem here
	public RightPositionLinear() {
		super("RightPositionLinear", Kp, Ki, Kd);
		getPIDController().disable();
		getPIDController().setPercentTolerance(5);

		// Use these to get going:
		// setSetpoint() - Sets where the PID controller should move the system
		// to
		// enable() - Enables the PID controller.
	}

	public void setPIDF(double Kp, double Ki, double Kd, double Kf) {
		getPIDController().setPID(Kp, Ki, Kd, Kf);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	protected double returnPIDInput() {
		return RobotMap.rightEncoder.getDistance();
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
	}

	protected void usePIDOutput(double output) {
		// RobotMap.driveRightMotor1.set(output);
		Robot.positionRightOutput = output;
		// Use output to drive your system, like a motor
		// e.g. yourMotor.set(output);
	}

	public void enablePID() {
		getPIDController().enable();

	}

	public void disablePID() {
		getPIDController().disable();
	}

	public void setSetpoint(double setpoint) {
		getPIDController().setSetpoint(setpoint);
	}

	public void setMaxOut(double speed) {
		getPIDController().setOutputRange(-speed, speed);
	}

	public boolean inPosition(double inPositionBand) {
		return (Math.abs(getPIDController().getSetpoint()
				- RobotMap.rightEncoder.getDistance()) < inPositionBand);
	}

	public boolean isEnabled() {
		return getPIDController().isEnabled();
	}

	public void updateStatus() {
		if (Robot.debugRightLinear) {
			SmartDashboard.putBoolean("Right Linear PID Enabled", isEnabled());
			SmartDashboard.putNumber("Right Linear Error", getPIDController()
					.getError());
			SmartDashboard.putNumber("Right Linear Output", getPIDController()
					.get());
			SmartDashboard.putNumber("Lin PID Right OUT",
					Robot.positionRightOutput);
		}
	}
}
