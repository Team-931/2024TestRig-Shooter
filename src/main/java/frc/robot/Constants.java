// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int opStickPort = 0;
  }
  public static class ClimberConstants {
  
    public static final int leftID = 11, rightID = 12;
  }
  public static class DriveConstants {
    public static final int driveIDs[] = {1,3,5,7},
                            turnIDs[]  = {2,4,6,8};
    
  }
  public static class IntakeConstants {
  
    
   public static final int intakeID = 15;
  }
  public static class  ArmConstants {
    public static final double gearing = 6 /* sprocket */ * 100 /* gearbox */ /1;
    public static final int ArmID = 10;
    public static final double holdAt0 = 8 /* volts */; // test Voltage to stay up at moment arm horizontal
    public static final double lowerLimit = 0;
    public static final double upperLimit = .25 /* rotation */;
    public static final double kP = 12. /* volts */ / 1/* rotation */;
  }
  public static class ShooterConstants {
    public static final double shootGearing = 2./3;
    
    public static final double shootTopSpd = -60*shootGearing,
      shootBottomSpd = -60*shootGearing,
      holdBackSpd = .3, holdFrontSpd = -.3, holdFwd = 1, holdRvs = -1;

    public static final int shootBottomID = 22, shootTopID = 23,
      holdBackID = 20, holdFrontID = 21,
      sensorID = 0;
    public static final double RpM2RpS = 1.0/60;
  }
}
