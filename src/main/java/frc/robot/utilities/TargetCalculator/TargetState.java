package frc.robot.utilities.TargetCalculator;

public class TargetState {
    
    public final double targetYaw;
    public final double targetHoodAngle;
    public final double[] targetVelocities;

    public TargetState(double yaw, double hoodAngle, double[] velocities) {
        this.targetYaw = yaw;
        this.targetHoodAngle = hoodAngle;
        this.targetVelocities = velocities;
    }
}
