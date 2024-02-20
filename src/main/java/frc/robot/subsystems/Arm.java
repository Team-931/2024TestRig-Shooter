package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm  extends SubsystemBase{
    public Arm() {
        FeedbackConfigs sensConfigs = new FeedbackConfigs();
        sensConfigs.FeedbackRotorOffset = angle.getValueAsDouble();
        sensConfigs.SensorToMechanismRatio = ArmConstants.gearing;
        motor.getConfigurator().apply(sensConfigs);
    }

    @Override
    public void periodic() {
        if (periodicdelay > 0) --periodicdelay;
      else {
        periodicdelay = 10;
        SmartDashboard.putString("arm angle", angle.refresh().toString());
        
      }
    }

    public double getAngle() {
        return angle.refresh().getValueAsDouble();
    }
    public void gotoAngle(double angle) {
        motor.setControl(angleOut.withPosition(angle));
    }
    private final TalonFX motor = new TalonFX(ArmConstants.ArmID);
    private final StatusSignal<Double> angle = motor.getPosition();
    private final PositionVoltage angleOut = new PositionVoltage(0);
    /**  used only in periodic() */
    private int periodicdelay = 0;
}
