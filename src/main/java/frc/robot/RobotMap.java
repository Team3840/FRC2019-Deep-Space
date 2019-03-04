/*----------------------------------------------------------------------------*/
/* FRC 7068                                                                   */
/* Deep Space 2019 RobotMap                                                   */
/*                                                                            */
/* Created: 02/10/2019                                                        */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  //Light Controller
  public static Spark lightController;
  // Pigeon IMU
  public static PigeonIMU pidgeyIMU;
  //Drive Train Left
  public static WPI_TalonSRX driveTalonLeft;
  public static WPI_VictorSPX driveVictorLeft1;
  public static WPI_VictorSPX driveVictorLeft2;
  //Drive Train Right
  public static WPI_TalonSRX driveTalonRight;
  public static WPI_VictorSPX driveVictorRight1;
  public static WPI_VictorSPX driveVictorRight2;
  //Actuactors
  public static WPI_VictorSPX intakeBallMotor;
  public static WPI_VictorSPX elbowRightMotor;
  public static WPI_TalonSRX elbowLeftMotor;
  public static WPI_TalonSRX elevatorRightMotor;
  public static WPI_TalonSRX elevatorLeftMotor;
  public static WPI_TalonSRX wristMotor;
  public static Solenoid suctionCups;
  public static Compressor comp;
  public static DigitalInput limitSwitch;
  public static boolean _currentLimEn = true;
  public static AnalogInput distanceToWall;
  static double iaccum = 0;
  
	public static void init() {

    // Light Constuctors
    lightController = new Spark(8);

    // Pigeon IUM
    pidgeyIMU = new PigeonIMU(7);
    
    // Constuctors for Left Drive
    driveTalonLeft = new WPI_TalonSRX(1);//Master
		driveVictorLeft1 = new WPI_VictorSPX(2);//Slave
    driveVictorLeft2 = new WPI_VictorSPX(3);//Slave
		//Master Slave For Left Drive
		driveVictorLeft1.follow(driveTalonLeft);
    driveVictorLeft2.follow(driveTalonLeft);

    // Constuctors For Right Drive     
    driveTalonRight = new WPI_TalonSRX(4);//Master
		driveVictorRight1 = new WPI_VictorSPX(5);//Slave
    driveVictorRight2 = new WPI_VictorSPX(6);//Slave
		//Master Slave for Right Drive
	  driveVictorRight1.follow(driveTalonRight);
    driveVictorRight2.follow(driveTalonRight);
    driveTalonRight.setInverted(true);
    driveVictorRight1.setInverted(true);
    driveVictorRight2.setInverted(true);
    
    /* factory default values */
    driveTalonLeft.configFactoryDefault();
    driveVictorLeft1.configFactoryDefault();
    driveVictorLeft2.configFactoryDefault();
    driveTalonRight.configFactoryDefault();
    driveVictorRight1.configFactoryDefault();
    driveVictorRight2.configFactoryDefault();
  
    // Intake constructors
    intakeBallMotor = new WPI_VictorSPX(9);
    intakeBallMotor.setInverted(true);

    // Solenoid
    suctionCups = new Solenoid(2,0);

    //Limit Switch
    limitSwitch = new DigitalInput(0);

    //Distance to wall
    distanceToWall = new AnalogInput(0);

    /***************************************
		 *  Lift Motor Encoder Configuration
		 ****************************/
    //Elevator Constructors
    elevatorLeftMotor = new WPI_TalonSRX(12);
    elevatorRightMotor = new WPI_TalonSRX(11);
    //Sets up follower
    elevatorRightMotor.follow(elevatorLeftMotor);

    //Setups the encoder position sensor
    elevatorLeftMotor.configFactoryDefault();
    elevatorRightMotor.configFactoryDefault();

    /* first choose the sensor */
    elevatorLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    elevatorLeftMotor.setSensorPhase(false);
    elevatorLeftMotor.setInverted(true);
    elevatorRightMotor.setInverted(true);
    
    // Reset sensor position
    elevatorLeftMotor.setIntegralAccumulator(iaccum, 0, 10);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    elevatorLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    elevatorLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    /* set the peak and nominal outputs */
    elevatorLeftMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    elevatorLeftMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    elevatorLeftMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    elevatorLeftMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* set closed loop gains in slot0 - see documentation */
    elevatorLeftMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    elevatorLeftMotor.config_kF(0, 0.1481, Constants.kTimeoutMs);
    elevatorLeftMotor.config_kP(0, 2.0, Constants.kTimeoutMs);
    elevatorLeftMotor.config_kI(0, 0, Constants.kTimeoutMs);
    elevatorLeftMotor.config_kD(0, 0, Constants.kTimeoutMs);
    /* set acceleration and vcruise velocity - see documentation */
    elevatorLeftMotor.configMotionCruiseVelocity(200, Constants.kTimeoutMs);
    elevatorLeftMotor.configMotionAcceleration(200, Constants.kTimeoutMs);
    /* zero the sensor */
    elevatorLeftMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  
    //set Brake Mode for Elevator Motor
    elevatorLeftMotor.setNeutralMode(NeutralMode.Brake);
    elevatorRightMotor.setNeutralMode(NeutralMode.Brake);
    
    // Current Limiting for Elevator Motor
    elevatorLeftMotor.configPeakCurrentLimit(Constants.kPeakCurrentAmps, Constants.CurrentLimitingkTimeoutMs);
		elevatorLeftMotor.configPeakCurrentDuration(Constants.kPeakTimeMs, Constants.CurrentLimitingkTimeoutMs);
		elevatorLeftMotor.configContinuousCurrentLimit(Constants.kContinCurrentAmps, Constants.CurrentLimitingkTimeoutMs);
    elevatorLeftMotor.enableCurrentLimit(_currentLimEn); // Honor initial setting

    /****************************
     *  Wrist Motor Configuration                                                                                                                                                           
    ****************************/
    // Wrist Constructors
    wristMotor = new WPI_TalonSRX(10);

    /* first choose the sensor */
    wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    wristMotor.setSensorPhase(true);
    wristMotor.setInverted(true);

    // Reset sensor position
    wristMotor.setIntegralAccumulator(iaccum, 0, 10);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    /* set the peak and nominal outputs */
    wristMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    wristMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    wristMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    wristMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* set closed loop gains in slot0 - see documentation */
    wristMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    wristMotor.config_kF(0, 0.1481, Constants.kTimeoutMs);
    wristMotor.config_kP(0, 2.7, Constants.kTimeoutMs);
    wristMotor.config_kI(0, 0, Constants.kTimeoutMs);
    wristMotor.config_kD(0, 0, Constants.kTimeoutMs);
    /* set acceleration and vcruise velocity - see documentation */
    wristMotor.configMotionCruiseVelocity(200, Constants.kTimeoutMs);
    wristMotor.configMotionAcceleration(200, Constants.kTimeoutMs);
    /* zero the sensor */
    wristMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    //set Brake Mode for Wrist Motor
    wristMotor.setNeutralMode(NeutralMode.Brake);
   
      // Current Limiting for Wrist Motor
    wristMotor.configPeakCurrentLimit(Constants.kPeakCurrentAmps, Constants.CurrentLimitingkTimeoutMs);
		wristMotor.configPeakCurrentDuration(Constants.kPeakTimeMs, Constants.CurrentLimitingkTimeoutMs);
		wristMotor.configContinuousCurrentLimit(Constants.kContinCurrentAmps, Constants.CurrentLimitingkTimeoutMs);
    wristMotor.enableCurrentLimit(_currentLimEn); // Honor initial setting

    /****************************
		 * Elbow Motor Configurations
		 ****************************/
     // Arm constucters
    elbowLeftMotor = new WPI_TalonSRX(8);
    elbowRightMotor = new WPI_VictorSPX(7);
    
    elbowRightMotor.follow(elbowLeftMotor);

    elbowLeftMotor.clearStickyFaults(20);
    elbowRightMotor.clearStickyFaults(20);
 
    /* first choose the sensor */
    elbowLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    elbowLeftMotor.setSensorPhase(true);
    elbowLeftMotor.setInverted(false);
    elbowRightMotor.setInverted(true);

   
    // Reset sensor position
    elbowLeftMotor.setIntegralAccumulator(iaccum, 0, 10);
    
    /* Set relevant frame periods to be at least as fast as periodic rate */
    elbowLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    elbowLeftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    
    /* set the peak and nominal outputs */
    elbowLeftMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    elbowLeftMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    elbowLeftMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    elbowLeftMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    
    /* set closed loop gains in slot0 - see documentation */
    elbowLeftMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    elbowLeftMotor.config_kF(0, 0.1481, Constants.kTimeoutMs);
    elbowLeftMotor.config_kP(0, 2.0, Constants.kTimeoutMs);
    elbowLeftMotor.config_kI(0, 0, Constants.kTimeoutMs);
    elbowLeftMotor.config_kD(0, 0, Constants.kTimeoutMs);
    /* set acceleration and vcruise velocity - see documentation */
    elbowLeftMotor.configMotionCruiseVelocity(200, Constants.kTimeoutMs);
    elbowLeftMotor.configMotionAcceleration(200, Constants.kTimeoutMs);
    /* zero the sensor */
    elbowLeftMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

     //set Brake Mode for Elevator Motor
    elbowLeftMotor.setNeutralMode(NeutralMode.Brake);
    elbowRightMotor.setNeutralMode(NeutralMode.Brake);

      // Current limiting for Left Elbow Motor
    elbowLeftMotor.configPeakCurrentLimit(Constants.kPeakCurrentAmps, Constants.CurrentLimitingkTimeoutMs);
    elbowLeftMotor.configPeakCurrentDuration(Constants.kPeakTimeMs, Constants.CurrentLimitingkTimeoutMs);
    elbowLeftMotor.configContinuousCurrentLimit(Constants.kContinCurrentAmps, Constants.CurrentLimitingkTimeoutMs);
    elbowLeftMotor.enableCurrentLimit(_currentLimEn); // Honor initial setting
 
    SmartDashboard.putBoolean("Current Limit Enabled", _currentLimEn);
  }

}

