package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.commands.Shooter.HoldPositionAngleShooter;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AngleShooterMotor extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new HoldPositionAngleShooter());

	}

	public void setSpeedLimit(double voltage) {
		RobotMap.angleShooterMotor.configMaxOutputVoltage(voltage);
	}

	public void valSet(double value) {
		RobotMap.angleShooterMotor.set(value);
	}

	public void setPercentVbusMode() {
		if (isStopped()) {
			RobotMap.angleShooterMotor
					.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
			RobotMap.angleShooterMotor.set(0);
		}
	}

	public void enableControl() {
		RobotMap.angleShooterMotor.enableControl();
	}

	public void setPositionMode() {
		if (isStopped()) {
			RobotMap.angleShooterMotor
					.changeControlMode(CANTalon.TalonControlMode.Position);
			RobotMap.angleShooterMotor.set(RobotMap.angleShooterMotor
					.getPosition());
		}
	}

	public void setPIDFiZ(double p, double i, double d, double ff, double iZone) {
		RobotMap.angleShooterMotor.setPID(p, i, d);
		RobotMap.angleShooterMotor.setF(ff);
		RobotMap.angleShooterMotor.setIZone((int) iZone);
	}

	public void setAccels(double voltRampRate, double closedLoopRampRate) {
		RobotMap.angleShooterMotor.setCloseLoopRampRate(closedLoopRampRate);
		RobotMap.angleShooterMotor.setVoltageRampRate(voltRampRate);
	}

	public int getEncVelocity() {
		return RobotMap.angleShooterMotor.getEncVelocity();
	}

	public double getPosition() {
		return RobotMap.angleShooterMotor.getPosition();
	}

	public int getError() {
		return RobotMap.angleShooterMotor.getClosedLoopError();
	}

	public double getCurrent() {
		return RobotMap.angleShooterMotor.getOutputCurrent();
	}

	public boolean inPosition() {
		return (Math
				.abs(((RobotMap.angleShooterMotor.getSetpoint() - RobotMap.angleShooterMotor
						.getPosition()) * Robot.angleDegreesPerEncoderRev)) < 2);
	}

	public boolean isPositionMode() {
		return RobotMap.angleShooterMotor.getControlMode() == CANTalon.TalonControlMode.Position;
	}

	public boolean isPercentVbusMode() {
		return RobotMap.angleShooterMotor.getControlMode() == CANTalon.TalonControlMode.PercentVbus;
	}

	public boolean isStopped() {
		return (Math.abs(getEncVelocity()) < 50);
	}

	public void clearStickyFaults() {
		RobotMap.angleShooterMotor.clearStickyFaults();
	}

	public void enableRevLimitSwitch() {
		RobotMap.angleShooterMotor.enableReverseSoftLimit(true);

	}

	public void disableRevLimitSwitch() {
		RobotMap.angleShooterMotor.enableReverseSoftLimit(false);

	}

	public double shooterSpeedCalculated() {
		// looks up shooter speed from a table of speed to angles
		// table is from 30 degrees to 180 degrees in 10 degree steps 30 deg is
		// array[0] entry
		// this covers shooting backwards also
		// interpolation is done to 1 degree resolution inside the 10 degree
		// steps
		double currentShooterAngle;
		int currentShooterAngleInt;
		int currentShooterAngleIntDividedBy10;
		int oneDegreeValue;
		double interpolationValue;
		int tableIndex;
		int tableStartAngle = 30;
		currentShooterAngle = RobotMap.angleShooterMotor.getPosition()
				* Robot.angleDegreesPerEncoderRev;// example 125 degrees
		if (currentShooterAngle < 30 || currentShooterAngle > 170)
			currentShooterAngle = 150;
		currentShooterAngleInt = (int) currentShooterAngle;// 125
		currentShooterAngleIntDividedBy10 = currentShooterAngleInt / 10;// 12
		tableIndex = currentShooterAngleIntDividedBy10 - (tableStartAngle / 10);
		// 12 - (30/10) = 9 is 120 degrees
		oneDegreeValue = currentShooterAngleInt
				- (currentShooterAngleIntDividedBy10 * 10);// 125 - 120 = 5
		interpolationValue = (Robot.speedAngleArray[tableIndex + 1] - Robot.speedAngleArray[tableIndex])
				* oneDegreeValue / 10;// (array[10 is 130 deg] - array[9 is 120
										// deg])* 5/10
		return (Robot.speedAngleArray[tableIndex] + interpolationValue);
	}

	public void updateStatus() {
//		if (RobotMap.shooterHomeSwitch.getTriggerState())
//			this.disableRevLimitSwitch();
//		else {
//			this.enableRevLimitSwitch();
//			if (getPosition() * 60 != Robot.shooterAngleHomePosition)
//				RobotMap.angleShooterMotor
//						.setPosition(Robot.shooterAngleHomePosition
//								/ Robot.angleDegreesPerEncoderRev);

			if (RobotMap.angleShooterMotor.getPosition()
					* Robot.angleDegreesPerEncoderRev < 80)
				Robot.reverseShoot = true;
			else
				Robot.reverseShoot = false;
			SmartDashboard
					.putNumber(
							"Position Angle",
							(RobotMap.angleShooterMotor.getPosition() * Robot.angleDegreesPerEncoderRev));
			SmartDashboard.putNumber("Calc ShooterRPM",
					shooterSpeedCalculated());
			if (Robot.debugAngleShooterMotor) {

				SmartDashboard.putNumber("RPM Velocity Angle",
						RobotMap.angleShooterMotor.getSpeed());

				SmartDashboard.putNumber("Closed Loop Error Angle",
						RobotMap.angleShooterMotor.getClosedLoopError());
				SmartDashboard.putNumber("Motor Current Angle",
						RobotMap.angleShooterMotor.getOutputCurrent());
				SmartDashboard.putNumber("Bus Voltage Angle",
						RobotMap.angleShooterMotor.getBusVoltage());
				SmartDashboard.putNumber("Sensor Velocity Angle",
						RobotMap.angleShooterMotor.getSpeed());
				SmartDashboard.putNumber("Encoder Position Angle",
						RobotMap.angleShooterMotor.getPosition()
								* Robot.angleDegreesPerEncoderRev);
				SmartDashboard.putNumber("Motor Voltage Angle",
						RobotMap.angleShooterMotor.getOutputVoltage());
				SmartDashboard.putNumber("Motor Velocity RPM Angle",
						getEncVelocity() * 60
								/ Robot.encoderCountsPerRevShooterMotor);
				SmartDashboard.putNumber("Setpoint Angle",
						RobotMap.angleShooterMotor.getSetpoint());
				SmartDashboard.putNumber("PID Angle Output",
						RobotMap.angleShooterMotor.get());
				SmartDashboard.putBoolean("Stopped Angle", isStopped());
				SmartDashboard.putBoolean("In Position Angle", inPosition());
				// checkFaults();
				// updatefaults();
			}
		
	}
}
