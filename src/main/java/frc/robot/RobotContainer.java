// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Arm arm = new Arm();
  private final Climber climber = new Climber();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandGenericHID opStick =
      new CommandGenericHID(OperatorConstants.opStickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // testing only . . .
    /* arm.setDefaultCommand(
      arm.run(() -> {arm.gotoAngle(-opStick.getLeftY());}));
   */}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(shooter::exampleCondition)
    //    .onTrue(new HoldCommand(shooter, 1));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(shooter.exampleMethodCommand());
    /* leftBumper and rightBumper buttons: forward and reverse shooter hold */
      opStick.axisGreaterThan(OperatorConstants.intakeAxis, OperatorConstants.intakeTheshhold).and(new Trigger (shooter::sensorOff)) .onTrue(shooter.holdCommand(ShooterConstants.holdFwd)
          .andThen(intake.runIf(.3, () -> {
            var a = arm.atBottom();
            SmartDashboard.putBoolean("Arm test", a);
            return a;})))
        .or(
      opStick.axisLessThan(OperatorConstants.intakeAxis, -OperatorConstants.intakeTheshhold) .onTrue(shooter.holdCommand(ShooterConstants.holdRvs)
          .andThen(intake.runIf(-.3, () -> {
            var a = arm.atBottom();
            SmartDashboard.putBoolean("Arm test", a);
            return a;})))
        )
                                            .onFalse(shooter.holdCommand(0) // possible bug !!! Line 85 may counteract it
                                            .andThen(intake.runcommand(0)));
    
    /* y button: shooter shoot */
      opStick.button(1)
                  .onTrue(shooter.shootCommand(1)
                      .andThen( new WaitUntilCommand(shooter::shootFastEnough), 
                                shooter.holdCommand(ShooterConstants.holdFwd))) // possible bug !!! Line 79 may counteract it
                  .onFalse(shooter.shootCommand(0)
                      .andThen(shooter.holdCommand(0)));
      opStick.button(6)  .onTrue(shooter.shootCommand(1))
                                .onFalse(shooter.shootCommand(0));
      opStick.button(5)  .onTrue(shooter.holdCommand(ShooterConstants.holdFwd))
                                .onFalse(shooter.holdCommand(0));
      /* a button: arm up */
      opStick.button(2) .onTrue(arm.upCmd(true));
      opStick.button(7) .onTrue(arm.upCmd(false));

      /* x button: release climber */
      opStick.button(3) .and(opStick.button(4).negate()) .onTrue(climber.topOrBottomCommand(true))
        .or (
      /* b button: retract climber and stop */
      opStick.button(4) .and(opStick.button(3).negate()) .onTrue(climber.topOrBottomCommand(false)) 
        )
        .or (
      opStick.button(3) .and(opStick.button(4)) .onTrue(climber.windDownCommand())
        )
                          .onFalse(climber.stayPutCommand());

      new Trigger(() -> climber.currentHigh(true))
                  .onTrue(climber.runOnce(() -> climber.stayPut1(true)));
      new Trigger(() -> climber.currentHigh(false))
                  .onTrue(climber.runOnce(() -> climber.stayPut1(false)));
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; //Autos.exampleAuto(shooter);
  }

  public void simulationInit() {
    shooter.simulationInit();
  }
}
