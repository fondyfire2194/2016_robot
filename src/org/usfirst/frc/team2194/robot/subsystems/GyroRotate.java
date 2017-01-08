/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team2194.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

/**
 *
 * @author John
 */
public class GyroRotate extends PIDSubsystem {

	private static final double Kp = .01;
	private static final double Ki = 0.0;
	private static final double Kd = 0.0;

	public boolean gyroState = true;
	public double last_world_linear_accel_x;
	public double last_world_linear_accel_y;
	public double targetAngle = 0;
	double lastTime = 0;
	double lastPitch = 0;
	double pitchRate = 0;
	double pitchHistory[];
	int i;
	int j;
	double anglePositionError;

	// Initialize your subsystem here
	public GyroRotate() {
		super("GyroRotate", Kp, Ki, Kd);
		getPIDController().setContinuous(true);
		getPIDController().setInputRange(0, 360);
		getPIDController().disable();
		getPIDController().setPercentTolerance(.55);//360 *.55/100 = degrees
}

	public void setPIDF(double Kp, double Ki, double Kd, double Kf) {
		getPIDController().setPID(Kp, Ki, Kd, Kf);
	}


	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	protected double returnPIDInput() {
		return Robot.imu.pidGet();
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

	public void resetGyro() {
		Robot.imu.reset();
	}

	public double getGyroAngle() {
		return Robot.imu.getAngle();
	}

	public float getGyroYaw() {
		return Robot.imu.getYaw();
	}

	public double getGyroYawError() {
		double yawErr = getGyroYaw() - targetAngle;
		return yawErr;
	}

	public double getGyroYawComp() {
		return (getGyroYawError() * Robot.prefs.getDouble(
				"Drive Straight Gyro Kp", .05));
	}

	public boolean inPosition() {
		return getPIDController().onTarget();
	}

	public boolean isEnabled() {
		return getPIDController().isEnabled();
	}

	public double getRoll() {
		return (-(Robot.imu.getPitch() - Robot.initialPitchAngle));

	}

	public double getGyroRollComp() {
		return (getRoll() * Robot.prefs.getDouble("Drive Level Gyro Kp", .05));
	}
	public double getZAcceleration(){
		return Robot.imu.getWorldLinearAccelZ();
	}

	public double getPitch() {
		return -Robot.imu.getRoll() + Robot.initialRollAngle;
	}

	public double getPitchRate() {
		pitchRate = (getPitch() - lastPitch)
				/ (Timer.getFPGATimestamp() - lastTime);// degrees per
														// second
		lastTime = Timer.getFPGATimestamp();
		lastPitch = getPitch();
		return pitchRate;
	}

	public double getCompassHeading() {
		return Robot.imu.getCompassHeading();
	}

	public double getAngleError() {
		return getPIDController().getError();
	}

	public boolean isMoving() {
		return Robot.imu.isMoving();
	}

	public boolean collisionDetect() {

		double curr_world_linear_accel_x = Robot.imu.getWorldLinearAccelX();
		double currentJerkX = curr_world_linear_accel_x
				- last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = Robot.imu.getWorldLinearAccelY();
		double currentJerkY = curr_world_linear_accel_y
				- last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_y;

		return ((Math.abs(currentJerkX) > Robot.kCollisionThreshold_DeltaG) || (Math
				.abs(currentJerkY) > Robot.kCollisionThreshold_DeltaG));
	}

	public void updateStatus() {
		SmartDashboard.putBoolean("Gyro State", gyroState);
		SmartDashboard.putNumber("IMU_Yaw", Robot.imu.getYaw());
		SmartDashboard.putNumber("IMU_Pitch", getPitch());
		SmartDashboard.putNumber("IMU_Roll", getRoll());
		SmartDashboard.putBoolean("Gyro PID Enabled", isEnabled());
		SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
		SmartDashboard.putBoolean("IsMoving", Robot.imu.isMoving());
		SmartDashboard.putBoolean("IsRotating", Robot.imu.isRotating());
		SmartDashboard.putNumber("Pitch Rate", getPitchRate());
		SmartDashboard.putNumber("Target Gyro Comp Angle", targetAngle);
		SmartDashboard.putNumber("Angle Error", getAngleError());
		SmartDashboard.putBoolean("Orient On Target", getPIDController()
				.onTarget());

		if (Robot.debugGyroRotate) {
			SmartDashboard.putNumber("Orient Setpoint", getPIDController()
					.getSetpoint());
			SmartDashboard.putNumber("Orient Error", getPIDController()
					.getError());
			SmartDashboard.putNumber("Orient Output", getPIDController().get());
			SmartDashboard.putBoolean("IMU_Connected", Robot.imu.isConnected());
			SmartDashboard.putBoolean("Is Calibrating",
					Robot.imu.isCalibrating());
		}
	}

}
