/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author John
 */
public class VisionRotate extends PIDSubsystem {

	private static final double Kp = .002;
	private static final double Ki = 0.0001;
	private static final double Kd = 0.0;

	public boolean visionState = false;

	// Initialize your subsystem here
	public VisionRotate() {
		super("VisionRotate", Kp, Ki, Kd);
		getPIDController().disable();
		getPIDController().setPercentTolerance(1);
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
		// if (Robot.xCameraVal == 999) return -10;//need to change this later
		// else
		return -Robot.xCameraVal;
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
	}

	protected void usePIDOutput(double output) {
		Robot.positionLeftOutput = output;
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

	public boolean inPosition() {
		return (Math.abs(getPIDController().getSetpoint() - Robot.xCameraVal) <= 1);
	}

	public boolean netTablesOK() {
		return (Robot.xCameraVal <= 999);
	}

	public double getVisionComp() {
		double visionComp;
		if (!Robot.reverseShoot){
			visionComp = -(Robot.xCameraVal - Robot.prefs.getDouble(
				"X Fwd Target", Robot.xCameraVal)) * Robot.squareToGoalKp;
		}
		else{
			visionComp = -(Robot.xCameraVal - Robot.prefs.getDouble(
					"X Rev Target", Robot.xCameraVal)) * Robot.squareToGoalKp;
		}
		if (visionComp > .2)
			visionComp = .2;
		else if (visionComp < -.2)
			visionComp = -.2;
		if (!targetOnScreen())
			visionComp = 0;
		if (Robot.reverseShoot) return -visionComp;
		else return visionComp;
	}

	public double getActiveYTarget() {
		Robot.visionYFwdTarget = Robot.prefs.getDouble("Y Fwd Target", 0);
		Robot.visionYRevTarget = Robot.prefs.getDouble("Y Rev Target", 0);
		if (!Robot.reverseShoot) {
			Robot.visionYActiveTarget = Robot.visionYFwdTarget;
		} else {
			Robot.visionYActiveTarget = Robot.visionYRevTarget;
		}
		return Robot.visionYActiveTarget;
	}
	public double getActiveXTarget() {
		Robot.visionXFwdTarget = Robot.prefs.getDouble("X Fwd Target", 0);
		Robot.visionXRevTarget = Robot.prefs.getDouble("X Rev Target", 0);
		if (!Robot.reverseShoot) {
			Robot.visionXActiveTarget = Robot.visionXFwdTarget;
		} else {
			Robot.visionXActiveTarget = Robot.visionXRevTarget;
		}
		return Robot.visionXActiveTarget;
	}

	public boolean onTarget() {
		Robot.visionXFwdTarget = Robot.prefs.getDouble("X Fwd Target", 0);
		Robot.visionYFwdTarget = Robot.prefs.getDouble("Y Fwd Target", 0);
		Robot.visionXRevTarget = Robot.prefs.getDouble("X Rev Target", 0);
		Robot.visionYRevTarget = Robot.prefs.getDouble("Y Rev Target", 0);
		if (!Robot.reverseShoot) {
			Robot.visionXActiveTarget = Robot.visionXFwdTarget;
			Robot.visionYActiveTarget = Robot.visionYFwdTarget;
		} else {
			Robot.visionXActiveTarget = Robot.visionXRevTarget;
			Robot.visionYActiveTarget = Robot.visionYRevTarget;
		}
//		return Math.abs(Robot.xCameraVal - Robot.visionXActiveTarget) <= 2
//				&& Math.abs(Robot.yCameraVal - Robot.visionYActiveTarget) <= 4;
		return onTargetX() && onTargetY();
	}
	public boolean onTargetX() {
		Robot.visionXFwdTarget = Robot.prefs.getDouble("X Fwd Target", 0);
		Robot.visionXRevTarget = Robot.prefs.getDouble("X Rev Target", 0);
		if (!Robot.reverseShoot) {
			Robot.visionXActiveTarget = Robot.visionXFwdTarget;
		} else {
			Robot.visionXActiveTarget = Robot.visionXRevTarget;
		}
		return Math.abs(Robot.xCameraVal - Robot.visionXActiveTarget) <= 8; //was 7
	}
	public boolean onTargetY() {
		Robot.visionYFwdTarget = Robot.prefs.getDouble("Y Fwd Target", 0);
		Robot.visionYRevTarget = Robot.prefs.getDouble("Y Rev Target", 0);
		if (!Robot.reverseShoot) {
			Robot.visionYActiveTarget = Robot.visionYFwdTarget;
		} else {
			Robot.visionYActiveTarget = Robot.visionYRevTarget;
		}
		return Math.abs(Robot.yCameraVal - Robot.visionYActiveTarget) <= 20; //was 9 then most recently 12
	}


	public boolean targetOnScreen() {
		//return Robot.xCameraVal < 998 && Robot.yCameraVal < 998;
		return Robot.xCameraVal < 999;
	}

	public boolean targetMidScreenX() {
		return Math.abs(Robot.xCameraVal) < 20;
	}

	public boolean isEnabled() {
		return getPIDController().isEnabled();
	}

	public void updateStatus() {

		// NetworkTable computer vision data
		SmartDashboard.putNumber("Camera X Read", Robot.xCameraVal);
		SmartDashboard.putNumber("Camera Y Read", Robot.yCameraVal);
		SmartDashboard.putBoolean("Using Vision", Robot.isVisionOrienting);
		SmartDashboard.putBoolean("Vision Comp On", Robot.useGoalVision);
		SmartDashboard.putBoolean("Target On Screen", targetOnScreen());
		SmartDashboard.putBoolean("Vision on X Target", onTargetX());
		SmartDashboard.putBoolean("Vision on Y Target", onTargetY());
		SmartDashboard.putNumber("X Vision Target", Robot.visionXActiveTarget);
		SmartDashboard.putNumber("Y Vision Target", Robot.visionYActiveTarget);
		SmartDashboard.putBoolean("Vision on Target", onTarget());
		SmartDashboard.putBoolean("Vision Shoot Over Robot",
				Robot.reverseShoot);

	}
}
