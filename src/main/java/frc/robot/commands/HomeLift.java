/*----------------------------------------------------------------------------*/
/*  Command Homing the lift to position.                                      */
/*  FRC 7068                                                                  */
/*  Deep Space                                                                */
/*  Date: 02/19/2019                                                                          */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class HomeLift extends Command {
  public HomeLift() {
    // Use requires() here to declare subsystem dependencies
   requires(Robot.liftElevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.liftElevator.LiftHoming();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    if (Robot.liftElevator.IsMotoredHome == true) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
