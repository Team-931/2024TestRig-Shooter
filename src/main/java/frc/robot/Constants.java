// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class FalconMotorConstants {
    public static final double kFreeSpeedRpm = 6380, 
    stallCurrent = 257 /* Amp */,
    stallTorque = 4.69 /* Newton-m */;
  }
  public static class OperatorConstants {
    public static final int opStickPort = 0;
    public static final int intakeAxis = 1;
    public static final double intakeTheshhold = .25;
  }
  public static class ClimberConstants {
  
    public static final int leftID = 11, rightID = 12;
    public static final double gearbox = 12, gearing = gearbox / .75 /* in. diam */ / Math.PI;
    /** The maxHeight is physically 19.5 in. but the gearing is too large 
     * at the bottom: the diameter increases as the rope winds up.
     */
    public static final double maxHeight = 13.5 /* inch */;
    public static final double currentHigh = .95 * FalconMotorConstants.stallCurrent /* Amp */ / 12 /* Volt */;
    public static final double pulleyDiam = 1.25, robotWeight = 100, gravity = 9.80665 /* m/s^2*/,
          supportTorque = Units.inchesToMeters(pulleyDiam) / 2 * Units.lbsToKilograms(robotWeight) / 2 * gravity / gearbox,
          supportVoltage = supportTorque / FalconMotorConstants.stallTorque * 12 /* V */;
    public static final double kP = 2;
  }
  public static class DriveConstants {
    /* IDs are assigned clockwise from front left. */
    public static final int driveIDs[] = {1,3,5,7},
                            turnIDs[]  = {2,4,6,8};
    
  }
  public static class IntakeConstants {
  
    
   public static final int intakeID = 15;
  }
  public static class  ArmConstants {
    public static final double gearing = 5 /* sprocket */ * 100 /* gearbox */ /1;
    public static final int ArmID = 10,
          boreEncoderID = 0;
    public static final double rampTime = 1 /* sec */;
    public static final double holdAt0 = .45 /* volts */; // test Voltage to stay up at moment arm horizontal
    public static final double boreOffset = .662;
    public static final double lowerLimit = -14. /*degrees */ / 360 /* degrees / rotation */+ .07;
    public static final double upperLimit = .31 /* rotation */;
    /** full power at 1/5 of full range i. e. >= 3 inches to go */
    public static final double kP = 12. /* volts */ / (upperLimit - lowerLimit) /* rotation */ * 5.;
    public static final double kD = /* kP * 0.04 */0;
  }
  public static class ShooterConstants {
    public static final double shootGearing = 2./3;
    
    public static final double shootTopSpd = -120*shootGearing,
      shootBottomSpd = 27.5 / 80 * shootTopSpd,
      holdBackSpd = .3, holdFrontSpd = -.3, holdFwd = 1, holdRvs = -1;

    public static final int shootBottomID = 22, shootTopID = 23,
      holdBackID = 20, holdFrontID = 21,
      sensorID = 0;
    public static final double RpM2RpS = 1.0/60;
  }
}
