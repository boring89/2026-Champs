package frc.robot.utilities.TargetCalculator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;
import java.util.Queue;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.RobotState.RobotState;

public class TargetCalculator extends SubsystemBase {

    private final Translation2d kBlueHub = new Translation2d(Units.inchesToMeters(182.1), Units.inchesToMeters(158.84));
    private final Translation2d kRedHub = new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));

    private final Translation2d kLeftBlueAlliance = new Translation2d(Meters.of(2), Meters.of(6));
    private final Translation2d kRightBlueAlliance = new Translation2d(Meters.of(2), Meters.of(2));

    private final Translation2d kLeftRedAlliance = new Translation2d(Meters.of(14.5), Meters.of(6));
    private final Translation2d kRightRedAlliance = new Translation2d(Meters.of(14.5), Meters.of(2));

    private final TargetMaps maps = new TargetMaps();

    private Pose2d predictedRobotPose = new Pose2d();

    private Rotation2d predictedTargetAngle = new Rotation2d();
    private double predictedTargetHoodPosition = 0;
    private AngularVelocity predictedTargetFlywheelVelocity = RotationsPerSecond.of(0);

    public enum Target {
        HUB,
        ALLIANCE
    }

    private RobotState robotState;

    public TargetCalculator(RobotState robotstate) {
        this.robotState = robotstate;
    }

    @Override
    public void periodic() {
        calculate();
    }

    public void calculate() {

        if (robotState.getPose() == null || robotState.getVelocity() == null) {
            Logger.recordOutput("TargetCalculator/isPresent", false);
            predictedTargetAngle = new Rotation2d();
            predictedTargetHoodPosition = 0.01;
            predictedTargetFlywheelVelocity = RotationsPerSecond.of(0);
            return;
        }

        Logger.recordOutput("TargetCalculator/isPresent", true);

        Optional<Alliance> nowAlliance = DriverStation.getAlliance();

        Translation2d targetTranslation = kBlueHub;

        Pose2d robotPose = robotState.getPose();
        ChassisSpeeds robotVelocity = robotState.getVelocity();

        Target target = Target.HUB;

        if (target == Target.HUB) {
            if (nowAlliance.isPresent()) {
                if (nowAlliance.get() == Alliance.Blue) {
                    targetTranslation = kBlueHub;
                } else {
                    targetTranslation = kRedHub;
                }
            } else {
                targetTranslation = kBlueHub;
            }
        } else {

            if (nowAlliance.isPresent()) {
                if (nowAlliance.get() == Alliance.Blue) {
                    if (robotPose.getY() > Units.inchesToMeters(158.845)) {
                        targetTranslation = kLeftBlueAlliance;
                    } else {
                        targetTranslation = kRightBlueAlliance;
                    }
                } else {
                    if (robotPose.getY() > Units.inchesToMeters(158.845)) {
                        targetTranslation = kRightRedAlliance;
                    } else {
                        targetTranslation = kLeftRedAlliance;
                    }
                }
            } else {
                if (robotPose.getY() > Units.inchesToMeters(158.845)) {
                    targetTranslation = kLeftBlueAlliance;
                } else {
                    targetTranslation = kRightBlueAlliance;
                }
            }
        }

        Translation2d predictedRobotTranslation = robotPose.getTranslation();
        Logger.recordOutput("TargetCalculator/PredictedRobotTranslationWithoutL",
                new Pose2d(predictedRobotTranslation, robotPose.getRotation()));

        // 迭代
        for (int i = 0; i < 5; i++) {
            double distance = targetTranslation.getDistance(predictedRobotTranslation);

            double timeOfFlight = maps.timeOfFlightMap.get(distance);

            Twist2d movementTwist2d = new Twist2d(
                    robotVelocity.vxMetersPerSecond * timeOfFlight,
                    robotVelocity.vyMetersPerSecond * timeOfFlight,
                    robotVelocity.omegaRadiansPerSecond * timeOfFlight);

            predictedRobotTranslation = robotPose.exp(movementTwist2d).getTranslation();
        }

        Distance predictedDistance = Meters.of(targetTranslation.getDistance(predictedRobotTranslation));

        predictedTargetAngle = targetTranslation.minus(predictedRobotTranslation).getAngle();
        predictedTargetFlywheelVelocity = maps.getFlyWheelVelocity(predictedDistance);
        predictedTargetHoodPosition = maps.getHoodPosition(predictedDistance);

        Logger.recordOutput("TargetCalculator/TargetFieldAngle", predictedTargetAngle.getRadians());
        Logger.recordOutput("TargetCalculator/TargetHoodPosition", predictedTargetHoodPosition);
        Logger.recordOutput("TargetCalculator/TargetFlywheelVelocity",
                predictedTargetFlywheelVelocity.in(RotationsPerSecond));

        Logger.recordOutput("TargetCalculator/PredictedRobotTranslation",
                new Pose2d(predictedRobotTranslation, robotPose.getRotation()));
        Logger.recordOutput("TargetCalculator/PredictedDistance", predictedDistance.in(Meters));
    }

    public Rotation2d getTargetAngle() {
        return predictedTargetAngle;
    }

    public double getTargetHoodPosition() {
        return predictedTargetHoodPosition;
    }

    public AngularVelocity getTargetFlywheelVelocity() {
        return predictedTargetFlywheelVelocity;
    }
}
