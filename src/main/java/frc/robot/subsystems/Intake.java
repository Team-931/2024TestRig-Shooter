package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final CANSparkMax intake = new CANSparkMax(IntakeConstants.intakeID, MotorType.kBrushless);
    public void periodic () {intake.set(-1 * running); }
    private double running = 0;
    public Command runcommand (double r) {
        return runOnce(
            () -> {running = r;});
    }
 
}
