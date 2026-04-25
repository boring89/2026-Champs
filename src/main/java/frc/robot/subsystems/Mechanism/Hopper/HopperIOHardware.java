package frc.robot.subsystems.Mechanism.Hopper;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Mechanism.DeviceIDs;

public class HopperIOHardware implements HopperIO {

    private final TalonFX motor;
    private VoltageOut output;
    private final StatusSignal<Voltage> voltage;

    public HopperIOHardware() {
        this.motor = new TalonFX(DeviceIDs.Hopper.MOTOR_ID, "rio");
        this.output = new VoltageOut(0);
        this.voltage = motor.getMotorVoltage();
    }


    @Override
    public void setVoltage(Voltage voltage) {
        motor.setControl(output.withOutput(voltage));
    }

    @Override
    public Voltage getVoltage() {
        this.voltage.refresh();
        return voltage.getValue();
    }

    @Override
    public void configure() {
        var config = new TalonFXConfiguration();

        config.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        config.CurrentLimits
                .withStatorCurrentLimit(50)
                .withSupplyCurrentLimit(30)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        
        motor.getConfigurator().apply(config);
    }

    @Override
    public Command setVoltageCommand(Voltage voltage) {
        return Commands.runOnce(() -> setVoltage(voltage));
    }
}
