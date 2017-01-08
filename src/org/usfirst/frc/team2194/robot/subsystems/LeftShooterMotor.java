package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LeftShooterMotor extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private int i;

	public void initDefaultCommand() {

		// Set the default command for a subsystem here.
	}

	public void valSet(double value) {
		RobotMap.leftShooterMotor.set(-value);
	}

	public void setPercentVbusMode() {
		if (isStopped()) {
			RobotMap.leftShooterMotor
					.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
			RobotMap.leftShooterMotor.set(0);
		}
	}

	public void enableControl() {
		RobotMap.leftShooterMotor.enableControl();
	}

	public void setSpeedMode() {
		if (isStopped()) {
			RobotMap.leftShooterMotor
					.changeControlMode(CANTalon.TalonControlMode.Speed);
			RobotMap.leftShooterMotor.set(0);
		}
	}

	public void setAccels(double voltRampRate, double closedLoopRampRate) {
		RobotMap.leftShooterMotor.setCloseLoopRampRate(closedLoopRampRate);
		RobotMap.leftShooterMotor.setVoltageRampRate(voltRampRate);
	}

	public int getEncVelocity() {
		return RobotMap.leftShooterMotor.getEncVelocity();
	}

	public double getPosition() {
		return RobotMap.leftShooterMotor.getPosition();
	}

	public int getError() {
		return RobotMap.leftShooterMotor.getClosedLoopError();
	}

	public boolean inPosition() {
		return (Math.abs(RobotMap.leftShooterMotor.get() - getPosition()) < 50);
	}

	public boolean isSpeedMode() {
		return RobotMap.leftShooterMotor.getControlMode() == CANTalon.TalonControlMode.Speed;
	}

	public boolean isStopped() {
		return (Math.abs(getEncVelocity()) < 50);
	}

	public void clearStickyFaults() {
		RobotMap.leftShooterMotor.clearStickyFaults();
	}

	public void updateStatus() {

		SmartDashboard.putNumber("RPM Velocity Left",
				RobotMap.leftShooterMotor.getSpeed());
		if (Robot.debugLeftShooterMotor) {
			SmartDashboard.putNumber("Motor Temp Left",
					RobotMap.leftShooterMotor.getTemperature());
			SmartDashboard.putNumber("Closed Loop Error Left",
					RobotMap.leftShooterMotor.getClosedLoopError());
			SmartDashboard.putNumber("Motor Current Left",
					RobotMap.leftShooterMotor.getOutputCurrent());
			SmartDashboard.putNumber("Bus Voltage Left",
					RobotMap.leftShooterMotor.getBusVoltage());
			// SmartDashboard.putNumber("Motor Velocity Left",
			// RobotMap.leftShooterMotor.getSpeed());
			SmartDashboard.putNumber("Encoder Position Left",
					RobotMap.leftShooterMotor.getPosition());
			SmartDashboard.putNumber("Motor Voltage Left",
					RobotMap.leftShooterMotor.getOutputVoltage());
			SmartDashboard.putNumber("Encoder Velocity Counts/Sec Left.",
					getEncVelocity() / 10);// method returns encoder counts per
											// 100ms.
			// SmartDashboard.putNumber("Motor Velocity RPM Left",
			// getEncVelocity() * 60/Robot.encoderCountsPerRevShooterMotor);
			// SmartDashboard.putBoolean("Position Mode", (isPositionMode()));
			// SmartDashboard.putBoolean("Speed Mode", (isSpeedMode()));
			// SmartDashboard.putBoolean("Pct Vbus Mode Mode",
			// (isPercentVbusMode()));
			SmartDashboard.putNumber("Setpoint Left",
					RobotMap.leftShooterMotor.getSetpoint());
			SmartDashboard.putNumber("PID Left Output",
					RobotMap.leftShooterMotor.get());
			SmartDashboard.putBoolean("Stopped Left", isStopped());
			SmartDashboard.putBoolean("In Position Left", inPosition());
			// SmartDashboard.putNumber("Motor Position Left",
			// RobotMap.leftShooterMotor.getPosition()/Robot.encoderCountsPerRevShooterMotor);
			// checkFaults();
			// updatefaults();
		}
	}
}
