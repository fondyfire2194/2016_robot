package org.usfirst.frc.team2194.robot.commands.ChoiceSelections;

import org.usfirst.frc.team2194.robot.Robot;
import org.usfirst.frc.team2194.robot.commands.CrossDefences.CrossChevalDeFriseGroup;
import org.usfirst.frc.team2194.robot.commands.CrossDefences.CrossDrawbridgeGroup;
import org.usfirst.frc.team2194.robot.commands.CrossDefences.CrossLowBarGroup;
import org.usfirst.frc.team2194.robot.commands.CrossDefences.CrossMoatGroup;
import org.usfirst.frc.team2194.robot.commands.CrossDefences.CrossPortcullisGroup;
import org.usfirst.frc.team2194.robot.commands.CrossDefences.CrossRampartsGroup;
import org.usfirst.frc.team2194.robot.commands.CrossDefences.CrossRockWallGroup;
import org.usfirst.frc.team2194.robot.commands.CrossDefences.CrossRoughTerrainGroup;
import org.usfirst.frc.team2194.robot.commands.CrossDefences.CrossSallyPortGroup;
import org.usfirst.frc.team2194.robot.commands.CrossDefences.DoNothingGroup;
import org.usfirst.frc.team2194.robot.commands.SlotMoves.DoNothingSlotGroup;
import org.usfirst.frc.team2194.robot.commands.SlotMoves.SlotMove1LeftGroup;
import org.usfirst.frc.team2194.robot.commands.SlotMoves.SlotMove2CenterGroup;
import org.usfirst.frc.team2194.robot.commands.SlotMoves.SlotMove2LeftGroup;
import org.usfirst.frc.team2194.robot.commands.SlotMoves.SlotMove3CenterGroup;
import org.usfirst.frc.team2194.robot.commands.SlotMoves.SlotMove4CenterGroup;
import org.usfirst.frc.team2194.robot.commands.SlotMoves.SlotMove5CenterGroup;
import org.usfirst.frc.team2194.robot.commands.SlotMoves.SlotMove5RightGroup;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SelectChoices extends Command {

	public SelectChoices() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.defenseSelected = (int) Robot.defenseType.getSelected();
		SmartDashboard.putNumber("Defense Selected", Robot.defenseSelected);

		switch (Robot.defenseSelected) {

		case 1:
			Robot.defenseCommand = new CrossPortcullisGroup();
			break;
		case 2:
			Robot.defenseCommand = (new CrossChevalDeFriseGroup());
			break;
		case 3:
			Robot.defenseCommand = (new CrossMoatGroup());
			break;
		case 4:
			Robot.defenseCommand = (new CrossRampartsGroup());
			break;
		case 5:
			Robot.defenseCommand = (new CrossDrawbridgeGroup());
			break;
		case 6:
			Robot.defenseCommand = (new CrossSallyPortGroup());
			break;
		case 7:
			Robot.defenseCommand = (new CrossRockWallGroup());
			break;
		case 8:
			Robot.defenseCommand = (new CrossRoughTerrainGroup());
			break;
		case 9:
			Robot.defenseCommand = (new CrossLowBarGroup());
			break;
		case 10:
			Robot.defenseCommand = (new DoNothingGroup());
			break;

		}
		Robot.defenseSlotSelected = (int) Robot.defenseSlot.getSelected();
		SmartDashboard.putNumber("Defense Slot Selected",
				Robot.defenseSlotSelected);

		switch (Robot.defenseSlotSelected) {
		case 1:
			Robot.slotMoveCommand = (new SlotMove1LeftGroup());
			break;
		case 2:
			Robot.slotMoveCommand = (new SlotMove2LeftGroup());
			break;
		case 3:
			Robot.slotMoveCommand = (new SlotMove2CenterGroup());
			break;
		case 4:
			Robot.slotMoveCommand = (new SlotMove3CenterGroup());
			break;
		case 5:
			Robot.slotMoveCommand = (new SlotMove4CenterGroup());
			break;
		case 6:
			Robot.slotMoveCommand = (new SlotMove5CenterGroup());
			break;
		case 7:
			Robot.slotMoveCommand = (new SlotMove5RightGroup());
			break;
		case 8:
			Robot.slotMoveCommand = (new DoNothingSlotGroup());
			break;

		}

		Robot.goalTargetSelected = (int) Robot.goalTarget.getSelected();
		SmartDashboard.putNumber("Goal Selected", (int)Robot.goalTargetSelected);

		switch (Robot.goalTargetSelected) {

		case 1:
			Robot.shootHighGoal = true;
			Robot.shootLowGoal = false;
			break;
		case 2:
			Robot.shootHighGoal = false;
			Robot.shootLowGoal = true;
			break;
		case 3:
			Robot.shootHighGoal = false;
			Robot.shootLowGoal = false;
			break;
		}

		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
