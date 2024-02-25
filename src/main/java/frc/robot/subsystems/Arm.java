package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm  extends SubsystemBase{
    public Arm() {
        var mctrl = motor.getConfigurator();
        mctrl.setPosition(ArmConstants.lowerLimit);
        var sensConfigs = new FeedbackConfigs();
        //sensConfigs.FeedbackRotorOffset = angle.getValueAsDouble();
        sensConfigs.SensorToMechanismRatio = ArmConstants.gearing;
        mctrl.apply(sensConfigs);
        var pid = new Slot0Configs()
             .withKP(ArmConstants.kP) 
             //.withKG(ArmConstants.holdAt0)
             .withGravityType(GravityTypeValue.Arm_Cosine);
        mctrl.apply(pid);
        var out = new MotorOutputConfigs()
             .withInverted(InvertedValue.Clockwise_Positive)
             .withNeutralMode(NeutralModeValue.Brake);
        mctrl.apply(out);
        var limitCfg = new HardwareLimitSwitchConfigs() 
            .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
            .withReverseLimitEnable(true);
        mctrl.apply(limitCfg);
        mctrl.apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(ArmConstants.rampTime));
    }

    @Override
    public void periodic() {
        if (periodicdelay > 0) --periodicdelay;
      else {
        periodicdelay = 10;
        SmartDashboard.putString("arm angle", angle.refresh().toString());
        SmartDashboard.putString("armVoltage", voltage.refresh().toString());
        SmartDashboard.putString("limit", limit.refresh().toString());
        
      }
    }

    public double getAngle() {
        return angle.refresh().getValueAsDouble();
    }
    public Command upCmd(boolean b) {
        return runOnce(() -> {gotoAngle(b ? ArmConstants.upperLimit: ArmConstants.lowerLimit);});
    }
    public void gotoAngle(double angle) {
        angle = Math.max(angle, ArmConstants.lowerLimit);
        angle = Math.min(angle, ArmConstants.upperLimit);
        SmartDashboard.putNumber("desired angle", angle);
        motor.setControl(angleOut.withPosition(angle));
    }
    private final TalonFX motor = new TalonFX(ArmConstants.ArmID);
    private final StatusSignal<Double> angle = motor.getPosition(), voltage = motor.getMotorVoltage();
    private final StatusSignal<ReverseLimitValue> limit = motor.getReverseLimit();
    private final PositionVoltage angleOut = new PositionVoltage(0) .withSlot(0);
    /**  used only in periodic() */
    private int periodicdelay = 0;
}
