package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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

    private double[] targetShooterAngularVelocity;
    private double targetHoodPosition;

    private AngularVelocity manualShooterAngularVelocity = RotationsPerSecond.of(50);
    private double manualHoodPosition;

    public Superstructure(
            CommandSwerveDrivetrain drivetrain,
            RollerIO roller,
            HopperIO hopper,
            FeederIO feeder,
            PivotIO pivot,
            ShooterIO shooter,
            HoodIO hood
            ) {
        this.drivetrain = drivetrain;
        this.roller = roller;
        this.hopper = hopper;
        this.feeder = feeder;
        this.pivot = pivot;
        this.shooter = shooter;
        this.hood = hood;

        this.robotState = new RobotState(drivetrain.getState());
        this.targetCalculator = new TargetCalculator(robotState);

        this.drivetrainPID = new PIDController(0.6, 0, 0.0);
        this.drivetrainPID.enableContinuousInput(-Math.PI, Math.PI);
        this.drivetrainPID.setTolerance(Units.degreesToRadians(2));
        this.targetShooterAngularVelocity = new double[3];
        this.targetHoodPosition = targetCalculator.getTargetHoodPosition();
        this.manualHoodPosition = 0.5;
    }

    // intake
    public Command intake() {
        return roller.setVoltageCommand(Volts.of(9)).alongWith(pivot.setAngleCommand(Positions.INTAKE));
    }

    public Command stopIntake() {
        return roller.setVoltageCommand(Volts.of(0));
    }

    public Command pivotDown() {
        return pivot.setAngleCommand(Positions.INTAKE);
    }

    public Command pivotBack() {
        return pivot.setAngleCommand(Positions.HALF);
    }

    public Command pivotShake() {
        return Commands.sequence(
                pivot.setAngleCommand(Positions.INTAKE).until(() -> pivot.isAtSetpoint()),
                pivot.setAngleCommand(Positions.HALF).until(()-> pivot.isAtSetpoint())).repeatedly();
    }

    public Command stopShake() {
        return pivot.setAngleCommand(Positions.INTAKE);
    }

    public Command shoot() {
        return shooter.enable();
    } 

    public Command stopShoot() {
        return shooter.disable();
    }

    public Command feed() {
        return Commands.parallel(
            feeder.setVoltageCommand(Volts.of(5.5)),
            hopper.setVoltageCommand(Volts.of(6))
        );
    }

    public Command stopFeed() {
        return Commands.parallel(
            feeder.setVoltageCommand(Volts.of(0)),
            hopper.setVoltageCommand(Volts.of(0))
        );
    }

    public Command outTake() {
        return Commands.parallel(
            feeder.setVoltageCommand(Volts.of(-5)),
            hopper.setVoltageCommand(Volts.of(-5)),
            pivot.setAngleCommand(Positions.INTAKE),
            roller.setVoltageCommand(Volts.of(-9))
        );
    }

    public Command stopOutTake() {
        return Commands.parallel(
            feeder.setVoltageCommand(Volts.of(0)),
            hopper.setVoltageCommand(Volts.of(0)),
            roller.setVoltageCommand(Volts.of(0))
        );
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
                () -> pivot.setAngle(Positions.BACK),
                () -> pivot.setAngle(Positions.INTAKE));
    }

    public Command shooterTest() {
        return Commands.startEnd(
                () -> shooter.enable(),
                () -> shooter.disable());
    }

    public Command shooterSpeedUp() {
        return Commands.runOnce(
            () -> this.manualShooterAngularVelocity = manualShooterAngularVelocity.plus(RotationsPerSecond.of(0.1)));
        
    }

    public Command shooterSlowDown() {
        return Commands.runOnce(
            () -> this.manualShooterAngularVelocity = this.manualShooterAngularVelocity.minus(RotationsPerSecond.of(0.1))
        );
    }

    public Command HoodUp() {
        return Commands.runOnce(
            () -> manualHoodPosition += 0.01);
    }

    public Command HoodDown() {
        return Commands.runOnce(
            () -> manualHoodPosition -= 0.01);
    }

    @Override
    public void periodic() {
        targetShooterAngularVelocity = targetCalculator.getTargetFlywheelVelocity();

        targetShooterAngularVelocity[0] = manualShooterAngularVelocity.in(RotationsPerSecond);
        targetShooterAngularVelocity[1] = manualShooterAngularVelocity.in(RotationsPerSecond); 
        targetShooterAngularVelocity[2] = manualShooterAngularVelocity.in(RotationsPerSecond);

        hood.setPosition(targetCalculator.getTargetHoodPosition());
        shooter.setVelocity(
            targetShooterAngularVelocity
        );

        hood.setPosition(manualHoodPosition);
        shooter.setVelocity(manualShooterAngularVelocity);

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
