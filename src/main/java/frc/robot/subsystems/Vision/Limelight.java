package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.utilities.VisionHelper.LimelightHelpers;

public class Limelight extends SubsystemBase {

    private final CommandSwerveDrivetrain drive;
    private final String limelightName;

    private final Pigeon2 gyro;

    private int tagId = -1;

    public Limelight(
            CommandSwerveDrivetrain drive,
            String limelightName) {
        this.drive = drive;
        this.limelightName = limelightName;
        this.gyro = drive.getPigeon2();
    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation(
                this.limelightName,
                this.drive.getPose().getRotation().getDegrees(),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0);

        /** MegaTag2 result */

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (mt2 == null)
            return;

        tagId = (int) LimelightHelpers.getFiducialID(limelightName);

        /** fillters */

        if (mt2.tagCount == 0)
            return;


        double xyStds;
        double degStds;
        double avgDist = mt2.avgTagDist;

            if (mt2.tagCount >= 2) {
                xyStds = 0.5;
                degStds = Double.MAX_VALUE;
            } else {

                xyStds = 1.0 * (avgDist * avgDist);
                degStds = Double.MAX_VALUE;
            }

        drive.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds,
                VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

    }

    // 提供給外部使用的 Getter
    public int getTagId() {
        return tagId;
    }
}