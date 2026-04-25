package frc.robot.subsystems.Mechanism.Pivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public interface PivotIO {

    public void setAngle(Angle angleDegrees);

    public Angle getAngle();

    public boolean isAtSetpoint();

    public void configure();

    public Command setAngleCommand(Angle angleDegrees);

    public Command sysIdDynamic(Direction direction);

    public Command sysIdQuasistatic(Direction direction);
}
