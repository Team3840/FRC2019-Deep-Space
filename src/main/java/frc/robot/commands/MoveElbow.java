/*----------------------------------------------------------------------------*/
/* Deep Space 2019                                                            */
/* FRC Team 7068                                                              */
/* MoveElbow Command                                                          */
/* Date 2/19/2019                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 *  Command to run lift to cube pickup position.
 *  Calls move elevator subsystem
 */
public class MoveElbow extends Command {
    String MoveElbow;
	
    public MoveElbow(String positionToMove) {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.elbow);
        MoveElbow = positionToMove;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        
    	Robot.elbow.ElbowToPosition(MoveElbow, Robot.oi.actuatorJoyStick);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Robot.elbow.CheckForInposition();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
