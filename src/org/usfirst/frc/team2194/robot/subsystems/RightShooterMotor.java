package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RightShooterMotor extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private int i;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
	}

	public void valSet(double value) {
		RobotMap.rightShooterMotor.set(value);
	}

	public void enableControl() {
		RobotMap.rightShooterMotor.enableControl();
	}

	public void setSpeedMode() {
		if (isStopped()) {
			RobotMap.rightShooterMotor
					.changeControlMode(CANTalon.TalonControlMode.Speed);
			RobotMap.rightShooterMotor.set(0);
		}
	}

	public int getEncVelocity() {
		return RobotMap.rightShooterMotor.getEncVelocity();
	}

	public double getPosition() {
		return RobotMap.rightShooterMotor.getPosition();
	}

	public int getError() {
		return RobotMap.rightShooterMotor.getClosedLoopError();
	}

	public boolean inPosition() {
		return (Math.abs(RobotMap.rightShooterMotor.get() - getPosition()) < 50);
	}

	public boolean isSpeedMode() {
		return RobotMap.rightShooterMotor.getControlMode() == CANTalon.TalonControlMode.Speed;
	}

	public boolean isStopped() {
		return (Math.abs(getEncVelocity()) < 50);
	}

	public void clearStickyFaults() {
		RobotMap.rightShooterMotor.clearStickyFaults();
	}

	public void updateStatus() {

		SmartDashboard.putNumber("RPM Velocity Right",
				RobotMap.rightShooterMotor.getSpeed());
		if (Robot.debugRightShooterMotor) {
			SmartDashboard.putNumber("Motor Temp Right",
					RobotMap.rightShooterMotor.getTemperature());
			SmartDashboard.putNumber("Closed Loop Error Right",
					RobotMap.rightShooterMotor.getClosedLoopError());
			SmartDashboard.putNumber("Motor Current Right",
					RobotMap.rightShooterMotor.getOutputCurrent());
			SmartDashboard.putNumber("Bus Voltage Right",
					RobotMap.rightShooterMotor.getBusVoltage());
			// SmartDashboard.putNumber("Sensor Velocity Right",
			// RobotMap.rightShooterMotor.getSpeed());
			SmartDashboard.putNumber("Encoder Position Right",
					RobotMap.rightShooterMotor.getPosition());
			SmartDashboard.putNumber("Motor Voltage Right",
					RobotMap.rightShooterMotor.getOutputVoltage());
			// SmartDashboard.putNumber("Encoder Velocity Counts/Sec Right.",
			// getEncVelocity()/10);// method returns encoder counts per 100ms.
			// SmartDashboard.putNumber("Motor Velocity RPM Right",
			// getEncVelocity() * 60/Robot.encoderCountsPerRevShooterMotor);
			SmartDashboard.putBoolean("Speed Mode", (isSpeedMode()));
			SmartDashboard.putNumber("Setpoint Right",
					RobotMap.rightShooterMotor.getSetpoint());
			SmartDashboard.putNumber("PID Right Output",
					RobotMap.rightShooterMotor.get());
			SmartDashboard.putBoolean("Stopped Right", isStopped());
			// SmartDashboard.putBoolean("In Position Right", inPosition());
			// SmartDashboard.putNumber("Motor Position Right",
			// RobotMap.rightShooterMotor.getPosition()/Robot.encoderCountsPerRevShooterMotor);
		}
	}
}
