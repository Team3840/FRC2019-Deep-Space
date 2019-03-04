/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 *  Intakes / Outtakes Cargo From Robot
 */
public class IntakeBall extends Subsystem {
  // Put methods for controlling this subsystem here. Call these from Commands.
  public final WPI_VictorSPX IntakeBallMotor = RobotMap.intakeBallMotor;
  
  //  SmartDashBoard , SuffleBoard 
	final String IntakeSpeed ="IntakeSpeed";
  final String OutTakeSpeed = "OutTakeSpeed";
  final double SpeedIn = 1.0;
	final double SpeedOut = -1.0;
  private  double setSpeed;

  @Override
  public void initDefaultCommand() {
    //setDefaultCommand(new );
    // Set the default command for a subsystem here.
  }

  public void intakeBall() {
    double backup = SpeedIn;
    setSpeed = getPreferencesDouble(IntakeSpeed ,backup);
    IntakeBallMotor.set(setSpeed);
  }

  public void ejectBall() {
    double backup = SpeedOut;
    setSpeed = getPreferencesDouble(OutTakeSpeed, backup);

    IntakeBallMotor.set(setSpeed);
  }
  
 public void stopMotion() {
     IntakeBallMotor.set(0.0);
 }


 /**
   	 * Retrieve numbers from the preferences table. If the specified key is in
   	 * the preferences table, then the preference value is returned. Otherwise,
   	 * return the backup value, and also start a new entry in the preferences
   	 * table. */
      private static double getPreferencesDouble(String key, double backup) {
        Preferences preferences = Preferences.getInstance();
        if(!preferences.containsKey(key)) {
          preferences.putDouble(key, backup);
        }
        return preferences.getDouble(key, backup);
          
        }
     
  }


