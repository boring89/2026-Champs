package frc.robot.subsystems.Mechanism.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterIOHardware extends SubsystemBase implements ShooterIO {

    private final TalonFX leftMotor, middleMotor, rightMotor;
    private final StatusSignal<AngularVelocity> leftVelocity, middleVelocity, rightVelocity;
    private final VelocityTorqueCurrentFOC out;
    private final TorqueCurrentFOC currentSysID;

    private final double gearRatio = 1;

    private AngularVelocity targetVelocity = RotationsPerSecond.of(0);

    public final SysIdRoutine sysIdRoutine;

    public boolean isShooting = false;

    public ShooterIOHardware() {
        this.leftMotor = new TalonFX(18, "canivore");
        this.middleMotor = new TalonFX(19, "canivore");
        this.rightMotor = new TalonFX(20, "canivore");

        this.leftVelocity = leftMotor.getVelocity();
        this.middleVelocity = middleMotor.getVelocity();
        this.rightVelocity = rightMotor.getVelocity();

        this.out = new VelocityTorqueCurrentFOC(0);
        this.currentSysID = new TorqueCurrentFOC(0);

        this.sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(1).per(Seconds),
                        Volts.of(10),
                        Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            leftMotor.setControl(currentSysID.withOutput(voltage.in(Volts)));
                        },
                        (log) -> {
                            log.motor("shooter_motor")
                                    .voltage(Volts.of(leftMotor.getTorqueCurrent().getValueAsDouble()))
                                    .angularPosition(leftMotor.getPosition().getValue())
                                    .angularVelocity(leftMotor.getVelocity().getValue());
                        },
                        this));

        configure(leftMotor, InvertedValue.CounterClockwise_Positive);
        configure(middleMotor, InvertedValue.Clockwise_Positive);
        configure(rightMotor, InvertedValue.Clockwise_Positive);
    }

    @Override
    public void periodic() {
        if (isShooting) {
            this.leftMotor.setControl(out.withVelocity(targetVelocity));
            this.middleMotor.setControl(out.withVelocity(targetVelocity));
            this.rightMotor.setControl(out.withVelocity(targetVelocity));
        }
    }

    @Override
    public Command enable() {
        return Commands.run(() -> isShooting = true);
    }

    @Override
    public Command disable() {
        return Commands.run(() -> isShooting = false);
    }

    @Override
    public void setVelocity(AngularVelocity velocityRPS) {
        this.targetVelocity = velocityRPS;
    }

    @Override
    public AngularVelocity[] getVelocity() {
        StatusSignal.refreshAll(leftVelocity, middleVelocity, rightVelocity);
        return new AngularVelocity[] { leftVelocity.getValue(), middleVelocity.getValue(), rightVelocity.getValue() };
    }

    @Override
    public boolean isAtSetpoint() {
        return Math.abs(leftVelocity.getValue().in(RotationsPerSecond) - targetVelocity.in(RotationsPerSecond)) < 1d
                && Math.abs(
                        middleVelocity.getValue().in(RotationsPerSecond) - targetVelocity.in(RotationsPerSecond)) < 1d
                && Math.abs(
                        rightVelocity.getValue().in(RotationsPerSecond) - targetVelocity.in(RotationsPerSecond)) < 1d;
    }

    @Override
    public void configure(TalonFX motor, InvertedValue invertDirection) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(80.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40);
        config.MotorOutput
                .withInverted(invertDirection)
                .withNeutralMode(NeutralModeValue.Coast);

        config.Slot0.kP = 10.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.Slot0.kV = 0.0;
        config.Slot0.kS = 0.0;

        config.Feedback.SensorToMechanismRatio = gearRatio;

        motor.getConfigurator().apply(config);
    }
}
