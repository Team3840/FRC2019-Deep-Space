/*----------------------------------------------------------------------------*/
/* Deep Space 2019                                                            */
/* FRC Team 7068                                                              */
/* Elbow Movements SubSystem                                                  */
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
 *  Elbow SubSystem
 */
public class Elbow extends Subsystem {
	//Hardware used.
	private final WPI_TalonSRX elbowMotor1 = RobotMap.elbowLeftMotor;
	
	// Used to get numbers from the smart dashboard perference values
	 final String ElbowHatchToHigh = "ElbowHatch_HL";
	 final String ElbowHatchToMid = "ElbowHatch_ML";
	 final String ElbowHatchToLow = "ElbowHatch_LL";
	 final String ElbowCargoToHigh = "ElbowCargo_HL";
	 final String ElbowCargoToMid = "ElbowCargo_ML";
	 final String ElbowCargoToLow = "ElbowCargo_LL";

	 final String ElbowToHatchPickup = "ElbowHatchPickUp";
	 final String ElbowToCargoPickup = "ElbowCargoPickup";
	 Boolean IsCargo;
	
	 //backup key values not returned from perference table on shuffleboard
	 final double HighCargo = 1.13;
	 final double HighHatch = 1.09;
	 final double MidCargo = 0.7;

	 final double MidHatch = 0.65;
	 final double LowCargo = 0.3;
	 final double LowHatch = 0.1;
	 final double CargoFloor = 0.1;//still need
	 final double HatchFloor = 0.37;
	
	 //local setpoint for moving to position by magic motion
	 private double setPoint;
	 int positionError;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	//setDefaultCommand(new LiftToManualPosition());
    }

    @Override
    public void periodic() {
    	//Gets the current closed loop error sensor position
    	positionError = elbowMotor1.getClosedLoopError(Constants.kPIDLoopIdx);
    	//pushes values to the dashboard
    	SmartDashboard.putNumber("ElbowSensor", elbowMotor1.getSelectedSensorPosition(Constants.kPIDLoopIdx));
		SmartDashboard.putNumber("ElbowOutputPercent", elbowMotor1.getMotorOutputPercent());
		SmartDashboard.putNumber("ElbowPositionError", elbowMotor1.getClosedLoopError(Constants.kPIDLoopIdx));
		SmartDashboard.putNumber("ElbowCurrentOutPut",elbowMotor1.getOutputCurrent());
    }

    /**
     * Public ElbowToPosition
     * Commands:  Cargo and Hatch locations on Rocket and floor pickup.
     *           
     */
    public void ElbowToPosition(String Key,XboxController _Joystick) {
		//grab joystick value
		IsCargo = _Joystick.getRawButton(7);
    	double backUp = LowCargo;
		SmartDashboard.putBoolean("IsCargo Elbow", IsCargo);
		
    	//set up the grab from values at Smart Dashboard perference table
    	 switch (Key) {
		 case ElbowHatchToHigh:;
		 //Hatch or Cargo
			if(IsCargo == true) {
				backUp = HighCargo;
			}
			else {
				backUp = HighHatch;
				Key = ElbowCargoToHigh;
			}
         	break;
		 case ElbowHatchToMid:;
			//Hatch or Cargo
			if(IsCargo == true) {
				backUp = MidCargo;
			}
			else {
				backUp = MidHatch;
			}
         	break;
		 case ElbowHatchToLow:;
			//Hatch or Cargo
			if(IsCargo == true) {
				backUp = LowCargo;
				Key = ElbowCargoToMid;
			}
			else {
				backUp = LowHatch;
			}
         	break;
         case ElbowToCargoPickup:;
         	backUp = CargoFloor;
			 break;
		 case ElbowToHatchPickup:;
         	backUp = HatchFloor;
			 break;
    	 }
    	 
    	//gets the current value
    	setPoint = getPreferencesDouble(Key, backUp);
    	
    	//dashboard/perference table sends rotations
    	setPoint = this.getValues(setPoint);      //setPoint * 4096
    			
    	/* Motion Magic - 4096 ticks/rev */
		elbowMotor1.set(ControlMode.MotionMagic,setPoint);
    }
    
    public void ResetEncoders() {
    	//Setups the encoder position sensor
    	elbowMotor1.clearStickyFaults(20);
        // Reset sensor position
    	elbowMotor1.setIntegralAccumulator(0, 0, 10);
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
