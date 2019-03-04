/*----------------------------------------------------------------------------*/
/* Deep Space 2019                                                            */
/* FRC Team 7068                                                              */
/* Wrist Movements SubSystem                                                  */
/* Date 2/19/2019                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;

/**
 *  Wrist SubSystem
 */
public class Wrist extends Subsystem {

  private final WPI_TalonSRX wristMotor= RobotMap.wristMotor;

  // Used to get numbers from the smart dashboard perference values
  final String WristHatchToHigh = "WristHatch_HL";
  final String WristHatchToMid = "WristHatch_ML";
  final String WristHatchToLow = "WristHatch_LL";
  final String WristCargoToHigh = "WristCargo_HL";
  final String WristCargoToMid = "WristCargo_ML";
  final String WristCargoToLow = "WristCargo_LL";

  final String WristToHatchPickup = "WristHatchPickUp";
  final String WristToCargoPickup = "WristCargoPickup";
  Boolean IsCargo;
 
  //backup key values not returned from perference table on shuffleboard
  final double HighCargo = -0.28;
  final double HighHatch = -0.1;
  final double MidCargo = -0.16;
  final double MidHatch = 0.065;
  final double LowCargo = -0.07;
  final double LowHatch = 0.24;
  final double CargoFloor = 0.1; //still need
  final double HatchFloor = -0.1;
 
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
    positionError = wristMotor.getClosedLoopError(Constants.kPIDLoopIdx);
    //pushes values to the dashboard
    SmartDashboard.putNumber("WristSensorPosition",wristMotor.getSelectedSensorPosition(Constants.kPIDLoopIdx));
    SmartDashboard.putNumber("WristOutputPercent", wristMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("WristPositionError", wristMotor.getClosedLoopError(Constants.kPIDLoopIdx));
    SmartDashboard.putNumber("WristCurrentOutPut", wristMotor.getOutputCurrent());
  }

  /**
   * Public LiftToPosition
   * Commands:  Wrist location Hatch / Cargo.
   *     
   */
  public void WristToPosition(String Key, XboxController _Joystick) {  
    //grab joystick value
	 	IsCargo = _Joystick.getRawButton(7);
    double backUp = LowCargo;
    SmartDashboard.putBoolean("IsCargo Wrist", IsCargo);
		    
      //set up the grab from values at Smart Dashboard perference table
      switch (Key) {
      case WristHatchToHigh:;
      //Hatch or Cargo
        if(IsCargo == true) {
          backUp = HighCargo;
          Key = WristCargoToHigh;
        }
        else {
          backUp = HighHatch;
        }
            break;
      case WristHatchToMid:;
        //Hatch or Cargo
        if(IsCargo == true) {
          backUp = MidCargo;
          Key = WristCargoToMid;
        }
        else {
          backUp = MidHatch;
        }
            break;
      case WristHatchToLow:;
        //Hatch or Cargo
        if(IsCargo == true) {
          backUp = LowCargo;
          Key = WristCargoToLow;
        }
        else {
          backUp = LowHatch;
        }
            break;
          case WristToCargoPickup:;
            backUp = CargoFloor;
        break;
      case WristToHatchPickup:;
            backUp = HatchFloor;
        break;
      }
     
    //gets the current value
    setPoint = getPreferencesDouble(Key, backUp);
    
    //dashboard/perference table sends rotations
    setPoint = this.getValues(setPoint);      //setPoint * 4096;
    
    /* Motion Magic - 4096 ticks/rev */
      wristMotor.set(ControlMode.MotionMagic,setPoint);
  }
  

  public void ResetEncoders() {
      //Setups the encoder position sensor
    wristMotor.clearStickyFaults(20);
        // Reset sensor position
    wristMotor.setIntegralAccumulator(0, 0, 10);
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
		 if (positionError >= +30){
			return true;
		 }
			
		/* Lower deadband */
		if (positionError <= -30)
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