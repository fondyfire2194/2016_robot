package org.usfirst.frc.team2194.robot.subsystems;

import org.usfirst.frc.team2194.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Manipulators extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void liftOneSetOff() {
		RobotMap.liftOne.set(DoubleSolenoid.Value.kOff);
	}

	public void liftOneRaise() {
		RobotMap.liftOne.set(DoubleSolenoid.Value.kForward);
	}

	public void liftOneLower() {
		RobotMap.liftOne.set(DoubleSolenoid.Value.kReverse);
	}

	public void liftOneToggle() {
		{
			if (RobotMap.liftOne.get() == DoubleSolenoid.Value.kReverse)
				liftOneRaise();
			else
				liftOneLower();
		}
	}

	public void liftTwoSetOff() {
		RobotMap.liftTwo.set(DoubleSolenoid.Value.kOff);
	}

	public void liftTwoRaise() {
		RobotMap.liftTwo.set(DoubleSolenoid.Value.kReverse);
	}

	public void liftTwoLower() {
		RobotMap.liftTwo.set(DoubleSolenoid.Value.kForward);
	}

	public void liftTwoToggle() {

		if (RobotMap.liftTwo.get() == DoubleSolenoid.Value.kReverse)
			liftTwoLower();
		else
			liftTwoRaise();
	}

	public void portcullisChevalManipulatorSetOff() {
		RobotMap.portcullisChevalManipulator.set(DoubleSolenoid.Value.kOff);
	}

	public void portcullisChevalManipulatorRaise() {
		RobotMap.portcullisChevalManipulator.set(DoubleSolenoid.Value.kReverse);
	}

	public void portcullisChevalManipulatorLower() {
		RobotMap.portcullisChevalManipulator.set(DoubleSolenoid.Value.kForward);
	}

	public void portcullisChevalManipulatorToggle() {
		if (RobotMap.portcullisChevalManipulator.get() == DoubleSolenoid.Value.kReverse)
			portcullisChevalManipulatorLower();
		else
			portcullisChevalManipulatorRaise();
	}
	public void updateStatus(){
		
	}
}
