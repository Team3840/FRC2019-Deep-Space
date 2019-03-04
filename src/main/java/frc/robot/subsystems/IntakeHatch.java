/*----------------------------------------------------------------------------*/
/* Deep Space 2019                                                            */
/* FRC Team 7068                                                              */
/* Hatch Intake/OutTake SubSystem                                             */
/* Date 2/19/2019                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Opens/Closes direct acting soleniod to release suction cups for the hatch.
 */
public class IntakeHatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final Solenoid suctionCups = RobotMap.suctionCups;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }
 
  public void SolenoidOn() {
    suctionCups.set(true);
  }

  public void SolenoidOff() {
   suctionCups.set(false);
  }
}
