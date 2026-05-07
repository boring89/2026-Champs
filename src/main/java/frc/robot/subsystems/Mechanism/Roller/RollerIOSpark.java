package frc.robot.subsystems.Mechanism.Roller;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RollerIOSpark implements RollerIO {

    private final SparkFlex motor;

    private double currentVolt = 0.0;

    public RollerIOSpark() {
        this.motor = new SparkFlex(16, MotorType.kBrushless);

        this.configure();
    }


    @Override
    public void setVoltage(Voltage voltage) {
        this.motor.setVoltage(voltage.in(Volts));
        this.currentVolt = voltage.in(Volts);
    }

    @Override
    public Voltage getVoltage() {
        return Volts.of(currentVolt);
    }

    @Override
    public void configure() {
        var config = new SparkFlexConfig();

        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(45);
            
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public Command setVoltageCommand(Voltage voltage) {
        return Commands.runOnce(() -> this.setVoltage(voltage));
    }
    
}
