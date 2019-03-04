/*----------------------------------------------------------------------------*/
/* Deep Space 2019                                                            */
/* FRC Team 7068                                                              */
/* Lift Movements SubSystem                                                   */
/* Date 2/19/2019                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 *  Lift Elevator Up/Down SubSystem
 */
public class LiftElevator extends Subsystem {

  private final WPI_TalonSRX liftMotor12 = RobotMap.elevatorLeftMotor;
  private final DigitalInput homeSwitch = RobotMap.limitSwitch;
  public Boolean IsMotoredHome;
 // private final XboxController actuatorJoyStick = Robot.oi.GetActuatorJoyStick();

  // Used to get numbers from the smart dashboard perference values
  final String LiftHatchHigh = "LiftHatch_HL";
  final String LiftHatchMid = "LiftHatch_ML";
  final String LiftHatchLow = "LiftHatch_LL";
  final String LiftCargoHigh = "LiftCargo_HL";
  final String LiftCargoMid = "LiftCargo_ML";
  final String LiftCargoLow = "LiftCargo_LL";

  final String LiftToHatchPickup = "LiftHatchPickUp";
  final String LiftToCargoPickup = "LiftCargoPickup";
  Boolean IsCargo;
 
  //backup key values not returned from perference table on shuffleboard
  final double HighCargo = 0.0;
  final double HighHatch = 0.0;
  final double MidCargo = 0.0;
  final double MidHatch = 0.0;
  final double LowCargo = 0.0;
  final double LowHatch = 0.0;
  final double CargoFloor = 0.1; //still need
  final double HatchFloor = 0.1; //still need
 
  //local setpoint for moving to position by magic motion
  private double setPoint;
  int positionError;

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new LiftToManualPosition());
  }

  @Override
  public void periodic() {
    //Gets the current sensorpositionErrorposition
    positionError = liftMotor12.getClosedLoopError(Constants.kPIDLoopIdx);
    //pushes values to the dashboard
    SmartDashboard.putNumber("LiftSensorPosition", liftMotor12.getSelectedSensorPosition(Constants.kPIDLoopIdx));
    SmartDashboard.putNumber("LiftOutputPercent", liftMotor12.getMotorOutputPercent());
    SmartDashboard.putNumber("LiftPositionError", liftMotor12.getClosedLoopError(Constants.kPIDLoopIdx));
    SmartDashboard.putNumber("LiftCurrentOutPut",liftMotor12.getOutputCurrent());
    
    //Check for limit switch.
    if (homeSwitch.get() == true) {
      liftMotor12.stopMotor();
    }
  }

  //Homes the lift up when teleop or auto init kicks off.
  public void LiftHoming() {
    IsMotoredHome = false;
    liftMotor12.set(ControlMode.PercentOutput, 0.2);

    //Resets the Encoder Position to zero / Stop motor
		if (homeSwitch.get() == true) {
      liftMotor12.set(ControlMode.PercentOutput, 0.0);
      liftMotor12.setSelectedSensorPosition(0);
      IsMotoredHome = true;
    }
  }

  /**
   * Public LiftToPosition
   * Commands:  Move to Hatch and Cargo Locations. 
   *            
   */
  public void LiftToPosition(String Key) {
     //grab joystick value
		//IsCargo = actuatorJoyStick.getRawButton(7);
    double backUp = LowCargo;
    
      //set up the grab from values at Smart Dashboard perference table
      switch (Key) {
      case LiftHatchHigh:;
      //Hatch or Cargo
        if(IsCargo = true) {
          backUp = HighCargo;
          Key = LiftCargoHigh;
        }
        else {
          backUp = HighHatch;
        }
            break;
      case LiftHatchMid:;
        //Hatch or Cargo
        if(IsCargo = true) {
          backUp = MidCargo;
          Key = LiftCargoMid;
        }
        else {
          backUp = MidHatch;
        }
            break;
      case LiftHatchLow:;
        //Hatch or Cargo
        if(IsCargo = true) {
          backUp = LowCargo;
          Key = LiftCargoLow;
        }
        else {
          backUp = LowHatch;
        }
            break;
          case LiftToCargoPickup:;
            backUp = CargoFloor;
        break;
      case LiftToHatchPickup:;
            backUp = HatchFloor;
        break;
      }
     
    //gets the current value
    setPoint = getPreferencesDouble(Key, backUp);
    
    //dashboard/perference table sends rotations
    setPoint = this.getValues(setPoint);      //setPoint * 4096;
            
    /* Motion Magic - 4096 ticks/rev */
       liftMotor12.set(ControlMode.MotionMagic,setPoint);
  }
  
   
  /**
   * getValues for commands (Ranges)
   * @param dblValue
   * @return new setpoint for lift position
   */
  private double getValues(double dblValue) {
    //Return the setpoint for moving lift
    dblValue = setPoint * 4096;
    return dblValue;
  }
  
  public boolean CheckForInposition() {
		/* Upper deadband */
		 if (positionError >= +300){
			return true;
		 }
			
		/* Lower deadband */
		if (positionError <= -300)
			return true;
		
		/* Outside deadband */
    return false;
    }

  /**
    * Retrieve numbers from the preferences table. If the specified key is in
    * the preferences table, then the preference value is returned. Otherwise,
    * return the backup value, and also start a new entry in the preferences
    * table.
    */
     private static double getPreferencesDouble(String key, double backup) {
     Preferences preferences = Preferences.getInstance();
     if (!preferences.containsKey(key)) {
       preferences.putDouble(key, backup);
     }
     return preferences.getDouble(key, backup);
   }
  
}