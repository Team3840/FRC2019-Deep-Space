/*----------------------------------------------------------------------------*/
/* Deep Space 2019                                                            */
/* FRC Team 7068                                                              */
/* CargoLowPosition Command Group                                            */
/* Date 2/19/2019                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CargoLowPosition extends CommandGroup {
  /**
   * Goto Cargo High Position
   */
  public CargoLowPosition() {
    // Add Commands here:

    // To run multiple commands at the same time,
    // use addParallel()
    addParallel(new MoveElbow("ElbowHatch_LL"));
    addParallel(new MoveWrist("WristCargo_LL"));
  }
}