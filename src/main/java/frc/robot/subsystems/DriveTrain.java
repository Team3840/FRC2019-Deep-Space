/*----------------------------------------------------------------------------*/
/* Deep Space 2019                                                            */
/* FRC Team 7068                                                              */
/* DriveTrain Subsystem                                                       */
/* Date 2/19/2019                                                             */
/* Revisions:                                                                 */
/* -Created 2/19/201                                                          */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveRobot;

/**
 * Drive Robot code!
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem here. Call these from Commands.
  private final WPI_TalonSRX  driveTalonL1 = RobotMap.driveTalonLeft;
  private final WPI_TalonSRX  driveTalonR1 = RobotMap.driveTalonRight;
  private final AnalogInput distanceToWall = RobotMap.distanceToWall;
  private final PigeonIMU pidgeyImu = RobotMap.pidgeyIMU;
  //Private variables
  private double [] xyz_dps = new double [3];
  private double forwardThrottle; 
  private	double turnThrottle;
  /** state for tracking whats controlling the drivetrain */
	enum GoStraight
	{
		Off, UsePigeon, SameThrottle
	};

  GoStraight _goStraight = GoStraight.Off;
  /*
	 * Some gains for heading servo, these were tweaked by using the web-based
	 * config (CAN Talon) and pressing gamepad button 6 to load them.
	 */
	double kPgain = 0.04; /* percent throttle per degree of error */
	double kDgain = 0.0004; /* percent throttle per angular velocity dps */
	double kMaxCorrectionRatio = 2.0; /* cap corrective turning throttle to 30 percent of forward throttle */
	/** holds the current angle to servo to */
  double _targetAngle = 0;
  /* nonzero to block the config until success, zero to skip checking */
  final int kTimeoutMs = 30;
  double currentAngle;
  boolean angleIsGood = (pidgeyImu.getState() == PigeonIMU.PigeonState.Ready) ? true : false;
  double currentAngularRate = xyz_dps[2];
  boolean userWantsGoStraight;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveRobot());
    //resets the pigeon
    pidgeyImu.setFusedHeading(0.0, kTimeoutMs); /* reset heading, angle measurement wraps at plus/minus 23,040 degrees (64 rotations) */
    _goStraight = GoStraight.Off; 
  }

  // Runs on periodic ~20ms updated with DS
  public void periodic() {
    //Gives driver a visual distance to the wall.
    double distance = (distanceToWall.getAverageVoltage()/512); //512 units per volt 
    SmartDashboard.putNumber("Distance", distance);
    //Get Pigeon status information from Pigeon API
    PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
    PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
    pidgeyImu.getGeneralStatus(genStatus);
    pidgeyImu.getRawGyro(xyz_dps);
    pidgeyImu.getFusedHeading(fusionStatus);
    currentAngle = fusionStatus.heading;
  }

	public void PercentOutput(XboxController joyStick1) {
    /* get input from gamepad to drive straight */
	  userWantsGoStraight = joyStick1.getRawButton(5); /* top left shoulder button */
    /* XboxController processing user inputs */
    forwardThrottle = joyStick1.getY() * -1.0; //Forward/Backward Speed
    turnThrottle = joyStick1.getX();	  //Turning Speed
    /* deadbands so centering joysticks always results in zero output */
		forwardThrottle = Deadband(forwardThrottle);
    turnThrottle = Deadband(turnThrottle);
    //Check for drive straight
    this.CheckForDriveStraight();
 }

 //Drive straight using pigeon or stay course!
 private void CheckForDriveStraight() {
   /* simple state machine to update our goStraight selection */
		switch (_goStraight) {

			/* go straight is off, better check gamepad to see if we should enable the feature */
			case Off:
				if (userWantsGoStraight == false) {
					/* nothing to do */
				} else if (angleIsGood == false) {
					/* user wants to servo but Pigeon isn't connected? */
					_goStraight = GoStraight.SameThrottle; /* just apply same throttle to both sides */
				} else {
					/* user wants to servo, save the current heading so we know where to servo to. */
					_goStraight = GoStraight.UsePigeon;
					_targetAngle = currentAngle;
				}
				break;

			/* we are servo-ing heading with Pigeon */
			case UsePigeon:
				if (userWantsGoStraight == false) {
					_goStraight = GoStraight.Off; /* user let go, turn off the feature */
				} else if (angleIsGood == false) {
					_goStraight = GoStraight.SameThrottle; /* we were servoing with pidgy, but we lost connection?  Check wiring and deviceID setup */
				} else {
					/* user still wants to drive straight, keep doing it */
				}
				break;

			/* we are simply applying the same throttle to both sides, apparently Pigeon is not connected */
			case SameThrottle:
				if (userWantsGoStraight == false) {
					_goStraight = GoStraight.Off; /* user let go, turn off the feature */
				} else {
					/* user still wants to drive straight, keep doing it */
				}
				break;
		}

		/* if we can servo with IMU, do the math here */
		if (_goStraight == GoStraight.UsePigeon) {
			/* very simple Proportional and Derivative (PD) loop with a cap,
			 * replace with favorite close loop strategy or leverage future Talon <=> Pigeon features. */
			turnThrottle = (_targetAngle - currentAngle) * kPgain - (currentAngularRate) * kDgain;
			/* the max correction is the forward throttle times a scalar,
			 * This can be done a number of ways but basically only apply small turning correction when we are moving slow
			 * and larger correction the faster we move.  Otherwise you may need stiffer pgain at higher velocities. */
			double maxThrot = MaxCorrection(forwardThrottle, kMaxCorrectionRatio);
			turnThrottle = Cap(turnThrottle, maxThrot);
		} else if (_goStraight == GoStraight.SameThrottle) {
			/* clear the turn throttle, just apply same throttle to both sides */
			turnThrottle = 0;
		} else {
			/* do nothing */
		}

		/* positive turnThrottle means turn to the left, this can be replaced with ArcadeDrive object, or teams drivetrain object */
		double left = forwardThrottle - turnThrottle;
		double right = forwardThrottle + turnThrottle;
		left = Cap(left, 1.0);
    right = Cap(right, 1.0);
     /* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
     driveTalonL1.set(ControlMode.PercentOutput, left);
     driveTalonR1.set(ControlMode.PercentOutput, right);
 }

  /** Deadband 5 percent, used on the gamepad */
  double Deadband(double value) {
    /* Upper deadband */
    if (value >= +0.05) 
      return value;
    
    /* Lower deadband */
    if (value <= -0.05)
      return value;
    
    /* Outside deadband */
    return 0;
  }
  
  /** @param value to cap.
	 * @param peak positive double representing the maximum (peak) value.
	 * @return a capped value.
	 */
	double Cap(double value, double peak) {
		if (value < -peak)
			return -peak;
		if (value > +peak)
			return +peak;
		return value;
  }
  
  /**
	 * Given the robot forward throttle and ratio, return the max
	 * corrective turning throttle to adjust for heading.  This is
	 * a simple method of avoiding using different gains for
	 * low speed, high speed, and no-speed (zero turns).
	 */
	double MaxCorrection(double forwardThrot, double scalor) {
		/* make it positive */
		if(forwardThrot < 0) {forwardThrot = -forwardThrot;}
		/* max correction is the current forward throttle scaled down */
		forwardThrot *= scalor;
		/* ensure caller is allowed at least 10% throttle,
		 * regardless of forward throttle */
		if(forwardThrot < 0.10)
			return 0.10;
		return forwardThrot;
	}

  // No motion...Stopped drive train.
  public void StopMotion() {
    driveTalonL1.set(ControlMode.PercentOutput, 0);
    driveTalonR1.set(ControlMode.PercentOutput, 0);
  }

 }
   
	

  

  

