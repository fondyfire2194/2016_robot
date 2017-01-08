package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.RobotMap;
import org.usfirst.frc.team2194.robot.commands.ControlCompressor;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AirCompressor extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new ControlCompressor());
	}

	public void start() {
		RobotMap.compressor.start();
	}

	public double readAirPressure() {
		return convertAnalogToPSI();
	}

	private double convertAnalogToPSI(){
		return (((RobotMap.airPressure.getAverageVoltage()/5)* 250) - 15);
	}

	public void stop() {
		RobotMap.compressor.stop();
	}
	public boolean isRunning(){
		return (!RobotMap.compressor.getPressureSwitchValue());
	}
//	public float getCurrentAmps(){
//		return RobotMap.compressor.getCompressorCurrent();
//	}
	public void updateStatus(){
		
		SmartDashboard.putBoolean("Compressor Running",isRunning());
		//SmartDashboard.putNumber("Compressor Amps", getCurrentAmps());
		SmartDashboard.putNumber("Analog PSI",readAirPressure());
		
	}
}
