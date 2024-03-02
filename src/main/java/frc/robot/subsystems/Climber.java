package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final TalonFX   leftMotor = new TalonFX(ClimberConstants.leftID), 
                            rightMotor = new TalonFX(ClimberConstants.rightID);
    private final PositionVoltage heightReq = new PositionVoltage(0);
    private final CoastOut coastOut = new CoastOut();
    private final StatusSignal<Double> ht = leftMotor.getPosition();

    public Climber() {
        var mctrl = leftMotor.getConfigurator();
        var sensConfigs = new FeedbackConfigs();
        sensConfigs.SensorToMechanismRatio = ClimberConstants.gearing;
        mctrl.apply(sensConfigs);
        mctrl.setPosition(0);
        var pid = new Slot0Configs()
             .withKP(/* ArmConstants.kP */1) 
             //.withKG(ArmConstants.holdAt0)
             .withGravityType(GravityTypeValue.Elevator_Static);
        mctrl.apply(pid);
        var out = new MotorOutputConfigs()
             .withInverted(InvertedValue.CounterClockwise_Positive)
             .withNeutralMode(NeutralModeValue.Brake);
        mctrl.apply(out);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        /* var limitCfg = new HardwareLimitSwitchConfigs() 
            .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
            .withReverseLimitEnable(true);
        mctrl.apply(limitCfg); */
        mctrl.apply(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(/* ArmConstants.rampTime */1));
        rightMotor.setControl(new Follower(ClimberConstants.leftID, true));
    }   
    
    public void gotoHeight(double height) {
        leftMotor.setControl(heightReq.withPosition(height));
    }

    public void coast() {
        leftMotor.setControl(coastOut);
    }

    public Command heightCommand (double height) {
        return startEnd(() -> {gotoHeight(height);}, () -> {gotoHeight(ht.refresh().getValueAsDouble());});
    }

    public Command stayPutCommand() {
        return runOnce(() -> {gotoHeight(ht.refresh().getValueAsDouble());});
    }
    public Command topOrBottomCommand(boolean isTop) {
        return heightCommand(isTop ? ClimberConstants.maxHeight : 0);
    }
    public Command coastCommand() {
        return runOnce(() -> {coast();});
    }
}
