package frc.robot.utilities.RobotState;


import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class RobotState extends SubsystemBase {

    public enum FieldState {
        BLUE_BOTTOM_ALLIANCE,
        BLUE_TOP_ALLIANCE,
        RED_BOTTOM_ALLIANCE,
        RED_TOP_ALLIANCE,
        MIDDLE,
        NULL
    }

    private final double kFieldLength = Units.inchesToMeters(651.22);
    private final double kFieldWidth = Units.inchesToMeters(317.69);
    private final double kAllianceAreaLength =  Units.inchesToMeters(182.11);
    private final double kBlueAllianceYaxis = Units.inchesToMeters(kAllianceAreaLength);
    private final double kRedAllianceYaxis = Units.inchesToMeters(kFieldLength - kAllianceAreaLength);


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
            return FieldState.NULL;
        }

        if (pose.getY() > kBlueAllianceYaxis) {
            if (pose.getX() < kFieldWidth / 2) {
                return FieldState.BLUE_BOTTOM_ALLIANCE;
            } else {
                return FieldState.BLUE_TOP_ALLIANCE;
            }
        } else if (pose.getY() < kRedAllianceYaxis) {
            if (pose.getX() < kFieldWidth / 2) {
                return FieldState.RED_BOTTOM_ALLIANCE;
            } else {
                return FieldState.RED_TOP_ALLIANCE;
            }
        } else {
            return FieldState.MIDDLE;
        }
        
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
