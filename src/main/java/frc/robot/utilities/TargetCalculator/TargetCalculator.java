package frc.robot.utilities.TargetCalculator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.RobotState.FieldState;
import frc.robot.utilities.RobotState.RobotState;
import frc.robot.utilities.RobotState.RobotState.AllianceState;
import frc.robot.utilities.RobotState.RobotState.LocationState;

public class TargetCalculator extends SubsystemBase {

    private final Translation2d kBlueHub = new Translation2d(Units.inchesToMeters(182.1), Units.inchesToMeters(158.84));
    private final Translation2d kRedHub = new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));

    private final Translation2d kLeftBlueAlliance = new Translation2d(Meters.of(2), Meters.of(6));
    private final Translation2d kRightBlueAlliance = new Translation2d(Meters.of(2), Meters.of(2));

    private final Translation2d kLeftRedAlliance = new Translation2d(Meters.of(14.5), Meters.of(6));
    private final Translation2d kRightRedAlliance = new Translation2d(Meters.of(14.5), Meters.of(2));

    private final TargetMaps maps = new TargetMaps();

    private Rotation2d predictedTargetAngle = new Rotation2d();
    private double predictedTargetHoodPosition = 0;
    private double[] predictedTargetFlywheelVelocity = new double[3];

    private RobotState robotState;

    public TargetCalculator(RobotState robotstate) {
        this.robotState = robotstate;
    }

    @Override
    public void periodic() {
        calculate();
    }

    public void calculate() {

        if (robotState.getPose() == null || robotState.getVelocity() == null || !DriverStation.getAlliance().isPresent()) {
            Logger.recordOutput("TargetCalculator/isPresent", false);
            Logger.recordOutput("TargetCalculator/ErrorMessage", "Robot pose or velocity is not available.");
            predictedTargetAngle = new Rotation2d();
            predictedTargetHoodPosition = 0.01;
            predictedTargetFlywheelVelocity[0] = 0.0;
            predictedTargetFlywheelVelocity[1] = 0.0;
            predictedTargetFlywheelVelocity[2] = 0.0;
            return;
        }

        Logger.recordOutput("TargetCalculator/ErrorMessage", "NONE");
        Logger.recordOutput("TargetCalculator/isPresent", true);

        Optional<Alliance> nowAlliance = DriverStation.getAlliance();

        Translation2d targetTranslation = kBlueHub;

        Pose2d robotPose = robotState.getPose();
        ChassisSpeeds robotVelocity = robotState.getVelocity();

        targetTranslation = this.selectTranslation(nowAlliance);

        Translation2d predictedRobotTranslation = robotPose.getTranslation();
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
        predictedTargetFlywheelVelocity[0] = maps.getLeftVelocity(predictedDistance);
        predictedTargetFlywheelVelocity[1] = maps.getMiddleVelocity(predictedDistance);
        predictedTargetFlywheelVelocity[2] = maps.getRightVelocity(predictedDistance);
        predictedTargetHoodPosition = maps.getHoodPosition(predictedDistance);

        Logger.recordOutput("TargetCalculator/TargetFieldAngle", predictedTargetAngle.getRadians());
        Logger.recordOutput("TargetCalculator/TargetHoodPosition", predictedTargetHoodPosition);
        // Logger.recordOutput("TargetCalculator/TargetFlywheelVelocity",
        //         predictedTargetFlywheelVelocity);

        Logger.recordOutput("TargetCalculator/PredictedDistance", predictedDistance.in(Meters));
    }

    public Rotation2d getTargetAngle() {
        return predictedTargetAngle;
    }

    public double getTargetHoodPosition() {
        return predictedTargetHoodPosition;
    }

    public double[] getTargetFlywheelVelocity() {
        return predictedTargetFlywheelVelocity;
    }

    private Translation2d selectTranslation(Optional<Alliance> nowAlliance) {

        FieldState state = robotState.getFieldState();
        
        AllianceState allianceState = state.allianceState;
        LocationState locationState = state.locationState;

        Alliance alliance = nowAlliance.isPresent() ? nowAlliance.get() : Alliance.Blue;

        boolean isBlue = alliance == Alliance.Blue;

        if (allianceState == AllianceState.NULL || locationState == LocationState.NULL) {
            return robotState.getPose().getTranslation();
        }

        if (locationState == LocationState.TOP) {
            if (allianceState.equals(AllianceState.BLUE)) {
                return isBlue ? kBlueHub : kLeftRedAlliance;
            } else if (allianceState.equals(AllianceState.RED)) {
                return !isBlue ? kRedHub : kLeftBlueAlliance;
            } else {
                return isBlue ? kLeftBlueAlliance : kLeftRedAlliance;
            }
        } else {
            if (allianceState.equals(AllianceState.BLUE)) {
                return isBlue ? kBlueHub : kRightRedAlliance;
            } else if (allianceState.equals(AllianceState.RED)) {
                return !isBlue ? kRedHub : kRightBlueAlliance;
            } else {
                return isBlue ? kRightBlueAlliance : kRightRedAlliance;
            }
        }
    } 
}
