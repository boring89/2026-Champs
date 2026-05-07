package frc.robot.subsystems.Mechanism.Feeder;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

public interface FeederIO {
    
    public void setVoltage(Voltage voltage);

    public Voltage getVoltage();

    public void configure();

    public Command setVoltageCommand(Voltage voltage);

    public Command stopMotor();
}
