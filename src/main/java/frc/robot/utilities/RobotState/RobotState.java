package frc.robot.utilities.RobotState;



import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotState extends SubsystemBase {

    public enum AllianceState {
        BLUE,
        RED,
        MIDDLE,
        NULL
    }

    public enum LocationState {
        TOP,
        BOTTOM,
        NULL
    }



    private final double kFieldLength = Units.inchesToMeters(651.22);
    private final double kFieldWidth = Units.inchesToMeters(317.69);
    private final double kAllianceAreaLength =  Units.inchesToMeters(182.11);
    private final double kBlueAllianceXaxis = kAllianceAreaLength;
    private final double kRedAllianceYaxis = kFieldLength - kAllianceAreaLength;


    private final SwerveDrivetrain.SwerveDriveState state;

    private Pose2d pose;
    private ChassisSpeeds velocity;
    private FieldState fieldState;

    public RobotState(SwerveDrivetrain.SwerveDriveState state) {
        this.state = state;
        pose = state.Pose;
        velocity = state.Speeds;
        fieldState = calculateFieldState();
    }

    @Override
    public void periodic() {
        pose = state.Pose;
        velocity = state.Speeds;
        fieldState = calculateFieldState();
    }

    private FieldState calculateFieldState() {

        if (pose == null) {
            return new FieldState(AllianceState.NULL, LocationState.NULL);
        }

        AllianceState alliance;
        LocationState location;

        if (pose.getX() < kBlueAllianceXaxis) {
            alliance = AllianceState.BLUE;
        } else if (pose.getX() > kRedAllianceYaxis) {
            alliance = AllianceState.RED;
        } else {
            alliance = AllianceState.MIDDLE;
        }

        if (pose.getY() > kFieldWidth / 2) {
            location = LocationState.TOP;
        } else {
            location = LocationState.BOTTOM;
        }
        
        SmartDashboard.putString("ALL", alliance.name());
        SmartDashboard.putString("LOC", location.name());
        return new FieldState(alliance, location);
    }

    public Pose2d getPose() {
        return pose;
    }

    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    public FieldState getFieldState() {
        return fieldState;
    }
}
