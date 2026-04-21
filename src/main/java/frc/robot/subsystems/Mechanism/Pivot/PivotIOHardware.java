package frc.robot.subsystems.Mechanism.Pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class PivotIOHardware extends SubsystemBase implements PivotIO {

    private static final double kPivotReduction = 50.0;
    private static final AngularVelocity kMaxPivotSpeed = RPM.of(6000).div(kPivotReduction);

    private final TalonFX motor;
    private final StatusSignal<Angle> angle;
    private final MotionMagicVoltage out;

    private final VoltageOut voltageSysId;

    private final SysIdRoutine sysIdRoutine;

    private Angle targetPositionRotations = Rotation.of(0);

    public PivotIOHardware() {
        this.motor = new TalonFX(14, "canivore");
        this.angle = motor.getPosition();
        this.out = new MotionMagicVoltage(0);
        this.voltageSysId = new VoltageOut(0);

        this.angle.refresh();
        if (angle.getValue() != null) {
            this.targetPositionRotations = angle.getValue();
        }

        this.sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(1).per(Seconds),
                        Volts.of(10),
                        Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            motor.setControl(voltageSysId.withOutput(voltage.in(Volts)));
                        },
                        (log) -> {
                            log.motor("pivot_motor")
                                    .voltage(Volts.of(motor.getTorqueCurrent().getValueAsDouble()))
                                    .angularPosition(motor.getPosition().getValue())
                                    .angularVelocity(motor.getVelocity().getValue());
                        },
                        this));
    }

    @Override
    public void periodic() {
        this.motor.setControl(this.out.withPosition(targetPositionRotations));
    }

    @Override
    public void setAngle(Angle angleDegrees) {
        this.targetPositionRotations = angleDegrees;
    
    }

    @Override
    public Angle getAngle() {
        return this.angle.getValue();
    }

    @Override
    public boolean isAtSetpoint() {
        return Math.abs(this.angle.getValue().in(Radian) - this.targetPositionRotations.in(Radian)) < 1;
    }

    @Override
    public void configure() {
        
       final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(300)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond))
                    .withGravityType(GravityTypeValue.Arm_Cosine)
            );

        motor.getConfigurator().apply(config);
    }
    
    @Override
    public Command setAngleCommand(Angle angleDegrees) {
        return Commands.runOnce(() -> setAngle(angleDegrees));
    }
}
