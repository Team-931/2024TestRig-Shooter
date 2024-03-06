package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
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
    private final StatusSignal<Double>  leftHt = leftMotor.getPosition(),
                                        rightHt= rightMotor.getPosition();

    public Climber() {
        var lctrl = leftMotor.getConfigurator();
        var rctrl = rightMotor.getConfigurator();
        var sensConfigs = new FeedbackConfigs();
        sensConfigs.SensorToMechanismRatio = ClimberConstants.gearing;
        lctrl.apply(sensConfigs);
        lctrl.setPosition(0);
        rctrl.apply(sensConfigs);
        rctrl.setPosition(0);
        var pid = new Slot0Configs()
             .withKP(/* ArmConstants.kP */1) 
             //.withKG(ArmConstants.holdAt0)
             .withGravityType(GravityTypeValue.Elevator_Static);
        lctrl.apply(pid);
        rctrl.apply(pid);
        var out = new MotorOutputConfigs()
             .withInverted(InvertedValue.Clockwise_Positive)
             .withNeutralMode(NeutralModeValue.Brake);
        lctrl.apply(out);
        rctrl.apply(out.withInverted(InvertedValue.CounterClockwise_Positive));
        /* var limitCfg = new HardwareLimitSwitchConfigs() 
            .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
            .withReverseLimitEnable(true);
        mctrl.apply(limitCfg); */
        var cc = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(/* ArmConstants.rampTime */1);
        lctrl.apply(cc);
        rctrl.apply(cc);
/*         rightMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        rightMotor.setControl(new Follower(ClimberConstants.leftID, true));
 */    }   
    
    public void gotoHeight(double height) {
        leftMotor.setControl(heightReq.withPosition(height));
        rightMotor.setControl(heightReq);
    }

    public void coast() {
        leftMotor.setControl(coastOut);
        rightMotor.setControl(coastOut);
    }

    public Command heightCommand (double height) {
        return runOnce(() -> {gotoHeight(height);});
    }

    public void stayPut() {
        leftMotor.setControl(heightReq.withPosition(leftHt.refresh().getValueAsDouble()));
        rightMotor.setControl(heightReq.withPosition(rightHt.refresh().getValueAsDouble()));
    }

    public Command stayPutCommand() {
        return runOnce(this::stayPut);
    }

    public Command topOrBottomCommand(boolean isTop) {
        return heightCommand(isTop ? ClimberConstants.maxHeight : 0);
    }
    public Command coastCommand() {
        return runOnce(() -> {coast();});
    }
}
