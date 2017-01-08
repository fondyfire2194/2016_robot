package org.usfirst.frc.team2194.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class PickShooterSpeed extends Subsystem {
    
	// table speeds are in rpm
	// table distance increment is in feet starting at baseDistanceIndex;
	// table angle increment is in degrees starting at baseAngleIndex
	public static int[][] speedTargetChoice;
	public static int baseAngleIndex = 40;//
	public static int baseDistanceIndex = 10;

	int myAngle;
	int myDistance;

	public PickShooterSpeed(double distance, double angle) {

		// table of speeds based on distance and angle settings
		// table starts at baseDistanceIndex and baseAngleIndex
		// need to subtract these from requested distance and angle
		// to get target speeds for these values

		int[][] speedTargetChoice = new int[10][10];
		myDistance = (int)distance;
		myAngle = (int)angle;
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

	public int getLowerTableSpeedTarget() {
		// casting double to int rounds down
		return speedTargetChoice[ myDistance - baseDistanceIndex][ myAngle
				- baseAngleIndex];

	}

	public int getUpperTableSpeedTarget() {
		// need to interpolate inside table settings
		//
		return speedTargetChoice[(myDistance - baseDistanceIndex + 1)] [(myAngle
				- baseAngleIndex + 1)];
	}

	public double getAverageSpeedTargetChoice() {

		return (double)((getUpperTableSpeedTarget() + getLowerTableSpeedTarget()) / 2);
	}
// Put methods for controlling this subsystem
    // here. Call these from Commands.


}

