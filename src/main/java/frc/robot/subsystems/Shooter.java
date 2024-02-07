// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Shooter() {

  }

  /**
   * Example command factory method.
   *
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
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //shootBottom.set(ShooterConstants.shootBottomSpd * shootIng);
    //shootTop.set(ShooterConstants.shootTopSpd * shootIng);
    holdBack.set(ShooterConstants.holdBackSpd * holdIng);
    holdFront.set(ShooterConstants.holdFrontSpd * holdIng);
  }

  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(holdBack, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(holdFront, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    REVPhysicsSim.getInstance().run();
  }

  private double holdIng = 0/* , shootIng = 0 */;

  public void setHold(double h) {
    holdIng = h;
  }
  
  static final VelocityDutyCycle shootVelocityDutyCycle = new VelocityDutyCycle (0);

  public void setShoot(double s) {
    shootTop.setControl(shootVelocityDutyCycle.withVelocity(s*ShooterConstants.shootTopSpd)
        .withFeedForward(Math.signum(s)));
    shootBottom.setControl(shootVelocityDutyCycle.withVelocity(s*ShooterConstants.shootBottomSpd)
        .withFeedForward(-Math.signum(s)));
  }

  private final TalonFX shootTop = new TalonFX(ShooterConstants.shootTopID),
   shootBottom = new TalonFX(ShooterConstants.shootBottomID);
  private final CANSparkMax holdFront = new CANSparkMax(ShooterConstants.holdFrontID, MotorType.kBrushless),
   holdBack = new CANSparkMax(ShooterConstants.holdBackID, MotorType.kBrushless);
}
