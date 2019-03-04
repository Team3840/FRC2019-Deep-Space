/*----------------------------------------------------------------------------*/
/* Deep Space 2019                                                            */
/* FRC Team 7068                                                              */
/* Robot OI                                                                   */
/* Date 2/19/2019                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Ball_In;
import frc.robot.commands.Ball_Out;
import frc.robot.commands.CargoHighPosition;
import frc.robot.commands.CargoLowPosition;
import frc.robot.commands.CargoMidPosition;
import frc.robot.commands.Hatch_Off;
import frc.robot.commands.MoveLift;
import frc.robot.commands.PickCargoFromFloor;
import frc.robot.commands.PickHatchFromFloor;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a joystick.

public XboxController driveJoyStick;
public XboxController actuatorJoyStick;

public OI () {
  //set constructors for controllers
  driveJoyStick = new XboxController(0);
  actuatorJoyStick = new XboxController(1);

  Button hatchOff = new JoystickButton(actuatorJoyStick, 8);
  hatchOff.whenPressed(new Hatch_Off());
  
  Button ballIn = new JoystickButton(actuatorJoyStick, 5);
  ballIn.whileHeld(new Ball_In());

  Button ballOut = new JoystickButton(actuatorJoyStick, 6);
  ballOut.whileHeld(new Ball_Out());

  Button HighPosition = new JoystickButton(actuatorJoyStick, 3);
  HighPosition.whenPressed(new CargoHighPosition());

  Button MidPosition = new JoystickButton(actuatorJoyStick, 2);
  MidPosition.whenPressed(new CargoMidPosition());

  Button LowPosition = new JoystickButton(actuatorJoyStick, 1);
  LowPosition.whenPressed(new CargoLowPosition());

  Button floorPositionHatch = new JoystickButton(driveJoyStick, 3);
  floorPositionHatch.whenPressed(new PickHatchFromFloor());

  Button floorPositionCargo = new JoystickButton(driveJoyStick, 2);
  floorPositionCargo.whenPressed(new PickCargoFromFloor());

 // Button moveElbow = new JoystickButton(actuatorJoyStick, 2);
  //moveElbow.whenPressed(new MoveElbow());

 // Button moveWrist = new JoystickButton(actuatorJoyStick, 4);
 // moveWrist.whenPressed(new MoveWrist());

  Button moveLift = new JoystickButton(actuatorJoyStick, 4);
  moveLift.whenPressed(new MoveLift("LiftHatchPickUp"));
  

  hatchOff.close();
  ballIn.close();
  ballOut.close();
  LowPosition.close();
  HighPosition.close();
  MidPosition.close();
  floorPositionCargo.close();
  floorPositionHatch.close();
  //moveElbow.close();
 // moveWrist.close();
  moveLift.close();

}

  public XboxController GetDriveJoyStick() {
    return driveJoyStick;
  }

  public XboxController GetActuatorJoyStick() {
    return actuatorJoyStick;
  }

}
