package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Mechanism.Feeder.FeederIO;
import frc.robot.subsystems.Mechanism.Hood.HoodIO;
import frc.robot.subsystems.Mechanism.Hopper.HopperIO;
import frc.robot.subsystems.Mechanism.Pivot.PivotIO;
import frc.robot.subsystems.Mechanism.Pivot.Positions;
import frc.robot.subsystems.Mechanism.Roller.RollerIO;
import frc.robot.subsystems.Mechanism.Shooter.ShooterIO;
import frc.robot.utilities.RobotState.RobotState;
import frc.robot.utilities.TargetCalculator.TargetCalculator;

public class Superstructure extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final RollerIO roller;
    private final HopperIO hopper;
    private final FeederIO feeder;
    private final PivotIO pivot;
    private final ShooterIO shooter;
    private final HoodIO hood;
    
    private final RobotState robotState;
    private final TargetCalculator targetCalculator;

    private final PIDController drivetrainPID;

    private final boolean isTest;

    private AngularVelocity targetShooterAngularVelocity;
    private double targetHoodPosition;

    private AngularVelocity manualShooterAngularVelocity;
    private double manualHoodPosition;

    public Superstructure(
            CommandSwerveDrivetrain drivetrain,
            RollerIO roller,
            HopperIO hopper,
            FeederIO feeder,
            PivotIO pivot,
            ShooterIO shooter,
            HoodIO hood,
            boolean isTest) {
        this.drivetrain = drivetrain;
        this.roller = roller;
        this.hopper = hopper;
        this.feeder = feeder;
        this.pivot = pivot;
        this.shooter = shooter;
        this.hood = hood;

        this.robotState = new RobotState(drivetrain.getState());
        this.targetCalculator = new TargetCalculator(robotState);

        this.drivetrainPID = new PIDController(0.5, 0, 0);
        this.drivetrainPID.enableContinuousInput(-Math.PI, Math.PI);

        this.isTest = isTest;

        this.targetShooterAngularVelocity = targetCalculator.getTargetFlywheelVelocity();
        this.targetHoodPosition = targetCalculator.getTargetHoodPosition();
        this.manualShooterAngularVelocity = RotationsPerSecond.of(0);
        this.manualHoodPosition = 0.5;
    }

    // intake
    public Command intake() {
        return Commands.sequence(
                pivot.setAngleCommand(Positions.INTAKE).until(pivot::isAtSetpoint),
                roller.setVoltageCommand(Volts.of(8)));
    }

    public Command stopIntake() {
        return roller.setVoltageCommand(Volts.of(0));
    }

    public Command pivotBack() {
        return Commands.parallel(
            roller.setVoltageCommand(Volts.of(0)),
            pivot.setAngleCommand(Positions.BACK)
        );
    }

    public Command pivotShake() {
        return Commands.sequence(
                pivot.setAngleCommand(Positions.INTAKE).withTimeout(0.5),
                pivot.setAngleCommand(Positions.HALF_BACK).withTimeout(0.5)).repeatedly();
    }

    public Command stopShake() {
        return pivot.setAngleCommand(Positions.INTAKE);
    }

    // launcher
    public Command shoot() {
        return Commands.sequence(
                shooter.enable().until(shooter::isAtSetpoint),
                hopper.setVoltageCommand(Volts.of(8)),
                feeder.setVoltageCommand(Volts.of(9)));
    }

    public Command stopShoot() {
        return Commands.parallel(
                shooter.disable(),
                feeder.setVoltageCommand(Volts.of(0)),
                hopper.setVoltageCommand(Volts.of(0)));
    }

    public double calculateDrivetrainOutput() {
        
        double output = drivetrainPID.calculate(
            robotState.getPose().getRotation().getRadians(), 
            targetCalculator.getTargetAngle().getRadians());

        return output;
    }

    // test
    public Command hopperTest() {
        return Commands.startEnd(
                () -> hopper.setVoltage(Volts.of(5)),
                () -> hopper.setVoltage(Volts.of(0)));
    }

    public Command feederTest() {
        return Commands.startEnd(
                () -> feeder.setVoltage(Volts.of(5)),
                () -> feeder.setVoltage(Volts.of(0)));
    }

    public Command rollerTest() {
        return Commands.startEnd(
                () -> roller.setVoltage(Volts.of(5)),
                () -> roller.setVoltage(Volts.of(0)));
    }

    public Command pivotTest() {
        return Commands.startEnd(
                () -> pivot.setAngle(Positions.HALF_BACK),
                () -> pivot.setAngle(Positions.INTAKE));
    }

    public Command shooterTest() {
        return Commands.startEnd(
                () -> shooter.enable(),
                () -> shooter.disable());
    }

    public Command hoodTest() {
        return Commands.startEnd(
                () -> hood.setPosition(0.5),
                () -> hood.setPosition(0));
    }

    @Override
    public void periodic() {
        targetShooterAngularVelocity = targetCalculator.getTargetFlywheelVelocity();
        targetHoodPosition = targetCalculator.getTargetHoodPosition();

        if (isTest) {
            shooter.setVelocity(manualShooterAngularVelocity);
            hood.setPosition(manualHoodPosition);
        } else {
            shooter.setVelocity(targetShooterAngularVelocity);
            hood.setPosition(targetHoodPosition);
        }
        dashboard();
    }

    private void dashboard() {
        Logger.recordOutput("Shooter/target", targetShooterAngularVelocity);
        Logger.recordOutput("Shooter/manual", manualShooterAngularVelocity);
        Logger.recordOutput("Shooter/actual", shooter.getVelocity()[1]);
        Logger.recordOutput("Hood/target", targetHoodPosition);
        Logger.recordOutput("Hood/manual", manualHoodPosition);
    }
}
