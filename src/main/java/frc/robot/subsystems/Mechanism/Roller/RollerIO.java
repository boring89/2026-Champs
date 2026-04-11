package frc.robot.subsystems.Mechanism.Roller;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

public interface RollerIO {

    public void setVoltage(Voltage voltage);

    public Voltage getVoltage();

    public void configure();

    public Command setVoltageCommand(Voltage voltage);
}
