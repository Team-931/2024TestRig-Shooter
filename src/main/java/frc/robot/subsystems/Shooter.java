// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter object. */
  public Shooter() {

  }

  /**
   * command factory method controlling the note shooting motors.
   *
   * @param s - "power" to pass on
   * @return a command
   */
  public Command shootCommand(double s) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          setShoot(s);
        });
  }

/**
   * command factory method controlling the note holding motors.
   *
   * @param h - "power" to pass on
   * @return a command
   */
  public Command holdCommand(double h) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          setHold(h);
          SmartDashboard.putNumber("holding:", h);
        });
  }

    /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
/*   public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
 */

    /** This method will be called once per scheduler run
     * (every 20 ms).
     */
    @Override
  public void periodic() {
    shootBottom.set(ShooterConstants.shootBottomSpd * shootIng);
    shootTop.set(ShooterConstants.shootTopSpd * shootIng);
    holdBack.set(ShooterConstants.holdBackSpd * holdIng);
    holdFront.set(ShooterConstants.holdFrontSpd * holdIng);
    {
      if (periodicdelay > 0) --periodicdelay;
      else {
        periodicdelay = 10;
        SmartDashboard.putString("shooter velocity", shVel.refresh().toString());
        SmartDashboard.putNumber("hold velocity (RpS)", holdEnc.getVelocity());
      }
    }
  }

/*   @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
 */
  private double holdIng = 0, shootIng = 0;

  public void setHold(double h) {
    holdIng = h;
  }
  public void setShoot(double s) {
    shootIng = s;
  }

  private final TalonFX shootTop = new TalonFX(ShooterConstants.shootTopID),
   shootBottom = new TalonFX(ShooterConstants.shootBottomID);
  private final CANSparkMax holdFront = new CANSparkMax(ShooterConstants.holdFrontID, MotorType.kBrushless),
   holdBack = new CANSparkMax(ShooterConstants.holdBackID, MotorType.kBrushless);
/**  used only in periodic() */
  private int periodicdelay = 0;
  private final StatusSignal<Double> shVel = shootTop.getVelocity();
  private final RelativeEncoder holdEnc = holdBack.getEncoder();
  {
    holdEnc.setVelocityConversionFactor(ShooterConstants.RpM2RpS);
  }
 }
