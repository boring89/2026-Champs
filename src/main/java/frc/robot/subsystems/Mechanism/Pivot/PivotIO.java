package frc.robot.subsystems.Mechanism.Pivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

public interface PivotIO {

    public void setAngle(Angle angleDegrees);

    public Angle getAngle();

    public boolean isAtSetpoint();

    public void configure();

    public Command setAngleCommand(Angle angleDegrees);
}
