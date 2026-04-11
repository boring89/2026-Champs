package frc.robot.utilities.TargetCalculator;

public class TargetState {
    
    public final double targetYaw;
    public final double targetHoodAngle;
    public final double targetFlywheelVelocity;

    public TargetState(double yaw, double hoodAngle, double flywheelVelocity) {
        this.targetYaw = yaw;
        this.targetHoodAngle = hoodAngle;
        this.targetFlywheelVelocity = flywheelVelocity;
    }
}
