/*----------------------------------------------------------------------------*/
/* Deep Space 2019                                                            */
/* FRC Team 7068                                                              */
/* CargoHighPosition Command Group                                            */
/* Date 2/19/2019                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PickCargoFromFloor extends CommandGroup {
  /**
   * Goto Cargo High Position
   */
  public PickCargoFromFloor() {
    // Add Commands here:

    // To run multiple commands at the same time,
    // use addParallel()
    addParallel(new MoveElbow("CargoPickup"));
    addParallel(new MoveWrist("CargoPickup"));
  }
}
