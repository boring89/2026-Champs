package frc.robot.subsystems.Mechanism.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Mechanism.DeviceIDs;

public class ShooterIOHardware extends SubsystemBase implements ShooterIO {

    private final TalonFX leftMotor, middleMotor, rightMotor;
    private final StatusSignal<AngularVelocity> leftVelocity, middleVelocity, rightVelocity;
    private final VelocityVoltage out;
    private final VoltageOut voltageSysID;

    private final double gearRatio = 1;

    private double[] targetVelocity = new double[3];

    public final SysIdRoutine sysIdRoutine;

    public boolean isShooting = false;

    public ShooterIOHardware() {
        this.leftMotor = new TalonFX(DeviceIDs.Shooter.LEFT_MOTOR_ID, "rio");
        this.middleMotor = new TalonFX(DeviceIDs.Shooter.MIDDLE_MOTOR_ID, "rio");
        this.rightMotor = new TalonFX(DeviceIDs.Shooter.RIGHT_MOTOR_ID, "rio");

        this.leftVelocity = leftMotor.getVelocity();
        this.middleVelocity = middleMotor.getVelocity();
        this.rightVelocity = rightMotor.getVelocity();

        this.out = new VelocityVoltage(0);
        this.voltageSysID = new VoltageOut(0);

        this.sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(1).per(Seconds),
                        Volts.of(10),
                        Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            leftMotor.setControl(voltageSysID.withOutput(voltage.in(Volts)));
                        },
                        (log) -> {
                            log.motor("shooter_motor")
                                    .voltage(Volts.of(leftMotor.getTorqueCurrent().getValueAsDouble()))
                                    .angularPosition(leftMotor.getPosition().getValue())
                                    .angularVelocity(leftMotor.getVelocity().getValue());
                        },
                        this));

        configure(leftMotor, InvertedValue.CounterClockwise_Positive);
        configure(middleMotor, InvertedValue.CounterClockwise_Positive);
        configure(rightMotor, InvertedValue.Clockwise_Positive);
    }

    @Override
    public void periodic() {

        Logger.recordOutput("Shooter/isShooting", this.isShooting);

        if (isShooting) {
            this.leftMotor.setControl(out.withVelocity(targetVelocity[0]));
            this.middleMotor.setControl(out.withVelocity(targetVelocity[1]));
            this.rightMotor.setControl(out.withVelocity(targetVelocity[2]));
        } else {
            this.leftMotor.stopMotor();
            this.middleMotor.stopMotor();
            this.rightMotor.stopMotor();
        }
    }

    @Override
    public Command enable() {
        return Commands.runOnce(() -> isShooting = true);
    }

    @Override
    public Command disable() {
        return Commands.runOnce(() -> isShooting = false);
    }

    @Override
    public void setVelocity(double[] velocityRPS) {
        this.targetVelocity = velocityRPS;
    }

    @Override
    public AngularVelocity[] getVelocity() {
        StatusSignal.refreshAll(leftVelocity, middleVelocity, rightVelocity);
        return new AngularVelocity[] { leftVelocity.getValue(), middleVelocity.getValue(), rightVelocity.getValue() };
    }

    @Override
    public boolean isAtSetpoint() {
        return Math.abs(leftVelocity.getValue().in(RotationsPerSecond) - targetVelocity[0]) < 5d
                && Math.abs(
                        middleVelocity.getValue().in(RotationsPerSecond) - targetVelocity[1]) < 5d
                && Math.abs(
                        rightVelocity.getValue().in(RotationsPerSecond) - targetVelocity[2]) < 5d;
    }

    @Override
    public void configure(TalonFX motor, InvertedValue invertDirection) {
         final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.5)
                    .withKI(2)
                    .withKD(0)
                    .withKV(12.0 / 100) // 12 volts when requesting max RPS
            );

        motor.getConfigurator().apply(config);
    }

    @Override
    public Command sysIdDynamic(Direction direction) {
        return sysIdDynamic(direction);
    }

    @Override
    public Command sysIdQuasistatic(Direction direction) {
        return sysIdQuasistatic(direction);
    }
}
