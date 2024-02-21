package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm  extends SubsystemBase{
    public Arm() {
        var mctrl = motor.getConfigurator();
        var sensConfigs = new FeedbackConfigs();
        sensConfigs.FeedbackRotorOffset = angle.getValueAsDouble();
        sensConfigs.SensorToMechanismRatio = ArmConstants.gearing;
        mctrl.apply(sensConfigs);
        var pid = new Slot0Configs() .withKP(12.) /* .withKI(12./10) */;
        mctrl.apply(pid);
        var out = new MotorOutputConfigs() .withInverted(InvertedValue.Clockwise_Positive) .withNeutralMode(NeutralModeValue.Brake);
        mctrl.apply(out);
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
        SmartDashboard.putNumber("desired angle", angle);
        motor.setControl(angleOut.withPosition(angle));
    }
    private final TalonFX motor = new TalonFX(ArmConstants.ArmID);
    private final StatusSignal<Double> angle = motor.getPosition();
    private final PositionVoltage angleOut = new PositionVoltage(0) .withSlot(0);
    /**  used only in periodic() */
    private int periodicdelay = 0;
}
