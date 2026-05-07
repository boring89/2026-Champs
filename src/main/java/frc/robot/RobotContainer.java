// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Dashboard.Dashboard;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Mechanism.Feeder.FeederIO;
import frc.robot.subsystems.Mechanism.Feeder.FeederIOHardware;
import frc.robot.subsystems.Mechanism.Hood.HoodIO;
import frc.robot.subsystems.Mechanism.Hood.HoodIOHardware;
import frc.robot.subsystems.Mechanism.Hopper.HopperIO;
import frc.robot.subsystems.Mechanism.Hopper.HopperIOHardware;
import frc.robot.subsystems.Mechanism.Pivot.PivotIO;
import frc.robot.subsystems.Mechanism.Pivot.PivotIOHardware;
import frc.robot.subsystems.Mechanism.Pivot.Positions;
import frc.robot.subsystems.Mechanism.Roller.RollerIO;
import frc.robot.subsystems.Mechanism.Roller.RollerIOHardware;
import frc.robot.subsystems.Mechanism.Roller.RollerIOSpark;
import frc.robot.subsystems.Mechanism.Shooter.ShooterIO;
import frc.robot.subsystems.Mechanism.Shooter.ShooterIOHardware;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.Vision.PhotonVision;
import frc.robot.utilities.FMS.Signal;

public class RobotContainer {

  private final CommandXboxController driverJoystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final SwerveRequest.FieldCentric normalDrive = new SwerveRequest.FieldCentric()
      .withDeadband(DrivetrainConstants.MaxDrivetrainSpeed.times(0.1))
      .withRotationalDeadband(DrivetrainConstants.MaxRotationSpeed.times(0.1))
      .withDesaturateWheelSpeeds(true)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentric targetAim = new SwerveRequest.FieldCentric()
      .withDeadband(DrivetrainConstants.MaxDrivetrainSpeed.times(0.1))
      .withRotationalDeadband(RotationsPerSecond.of(0.0))
      .withDesaturateWheelSpeeds(true)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final PivotIO pivot = new PivotIOHardware();
  private final RollerIO roller = new RollerIOSpark();
  private final HopperIO hopper = new HopperIOHardware();
  private final FeederIO feeder = new FeederIOHardware();
  private final ShooterIO shooter = new ShooterIOHardware();
  private final HoodIO hood = new HoodIOHardware();

  private final Superstructure superstructure = new Superstructure(
      drivetrain, roller, hopper, feeder, pivot, shooter, hood);

  private final Telemetry logger = new Telemetry(this.drivetrain.getState());

  private final SendableChooser<Command> autoChooser;

  public final Limelight limelight_Center = new Limelight(drivetrain, "limelight-center");
  // public final Limelight limelight_Left = new Limelight(drivetrain, "limelight-left");
  // public final Limelight limelight_Right = new Limelight(drivetrain, "limelight-right");

  public final Dashboard dashboard = new Dashboard(new Signal());

  // public final PhotonVision photonvision = new PhotonVision(drivetrain, PhotonVisionConstants.cameraTransforms);

  public RobotContainer() {
    
    NamedCommands.registerCommand("Intake", superstructure.intake().alongWith(superstructure.pivotDown()));
    NamedCommands.registerCommand("Stop_Intake", superstructure.stopIntake());
    NamedCommands.registerCommand("Shoot_With_Shake", 
      superstructure.shoot().withTimeout(1.0)
      .andThen(hopper.setVoltageCommand(Volts.of(5)))
      .alongWith(feeder.setVoltageCommand(Volts.of(5))).withTimeout(0.0001));

    NamedCommands.registerCommand("Stop_Shoot", 
      superstructure.stopShoot()
      .alongWith(hopper.stopMotor())
      .alongWith(feeder.stopMotor().withTimeout(0.001))
    );


    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Chooser", autoChooser);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          return normalDrive
              .withVelocityX(DrivetrainConstants.MaxDrivetrainSpeed.times(-driverJoystick.getLeftY())
                  .times(1))
              .withVelocityY(DrivetrainConstants.MaxDrivetrainSpeed.times(-driverJoystick.getLeftX())
                  .times(1))
              .withRotationalRate(DrivetrainConstants.MaxRotationSpeed.times(-driverJoystick.getRightX()));
        }));
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    drivetrain.registerTelemetry(logger::telemeterize);

    // mechanismTest();
    normalControls();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void normalControls() {
    driverJoystick.a().whileTrue(
        drivetrain.applyRequest(() -> {
          return targetAim
              .withVelocityX(DrivetrainConstants.MaxDrivetrainSpeed.times(-driverJoystick.getLeftY())
                  .times(0.75))
              .withVelocityY(DrivetrainConstants.MaxDrivetrainSpeed.times(-driverJoystick.getLeftX())
                  .times(0.75))
              .withRotationalRate(DrivetrainConstants.MaxRotationSpeed.times(superstructure.calculateDrivetrainOutput()));
        })
    );



    driverJoystick.leftTrigger().onTrue(superstructure.intake())
        .onFalse(superstructure.stopIntake());

    driverJoystick.leftBumper().onTrue(pivot.setAngleCommand(Positions.HALF))
    .onFalse(pivot.setAngleCommand(Positions.INTAKE));


    driverJoystick.rightTrigger().whileTrue(superstructure.shoot())
        .onFalse(superstructure.stopShoot());

    driverJoystick.rightBumper().onTrue(superstructure.feed())
        .onFalse(superstructure.stopFeed());

    // driverJoystick.povUp().onTrue(superstructure.shooterSpeedUp());

    // driverJoystick.povDown().onTrue(superstructure.shooterSlowDown());

    // driverJoystick.povLeft().onTrue(superstructure.HoodUp());

    // driverJoystick.povRight().onTrue(superstructure.HoodDown());

    driverJoystick.y().onTrue(feeder.setVoltageCommand(Volts.of(-5)))
    .onFalse(feeder.setVoltageCommand(Volts.of(0)));

    driverJoystick.povDown().onTrue(superstructure.outTake())
    .onFalse(superstructure.stopOutTake());
  }

  // public void sysidTest() {
  //   driverJoystick.povUp().and(driverJoystick.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
  //   driverJoystick.povUp().and(driverJoystick.b()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
  //   driverJoystick.povUp().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
  //   driverJoystick.povUp().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

  //   driverJoystick.povDown().and(driverJoystick.a()).whileTrue(shooter.sysIdDynamic(Direction.kForward));
  //   driverJoystick.povDown().and(driverJoystick.b()).whileTrue(shooter.sysIdDynamic(Direction.kReverse));
  //   driverJoystick.povDown().and(driverJoystick.x()).whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
  //   driverJoystick.povDown().and(driverJoystick.y()).whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));

  //   driverJoystick.povLeft().and(driverJoystick.a()).whileTrue(pivot.sysIdDynamic(Direction.kForward));
  //   driverJoystick.povLeft().and(driverJoystick.b()).whileTrue(pivot.sysIdDynamic(Direction.kReverse));
  //   driverJoystick.povLeft().and(driverJoystick.x()).whileTrue(pivot.sysIdQuasistatic(Direction.kForward));
  //   driverJoystick.povLeft().and(driverJoystick.y()).whileTrue(pivot.sysIdQuasistatic(Direction.kReverse));
  // }

  public void mechanismTest() {
    // driverJoystick.a().whileTrue(superstructure.intake());
    // driverJoystick.b().whileTrue(superstructure.hopperTest());
    // driverJoystick.x().whileTrue(superstructure.feederTest());
    // driverJoystick.y().whileTrue(superstructure.pivotTest());
    // driverJoystick.rightTrigger().whileTrue(superstructure.shooterTest());

    driverJoystick.povUp().onTrue(superstructure.shooterSpeedUp());
    driverJoystick.povDown().onTrue(superstructure.shooterSlowDown());
  }

  public Command aim(DoubleSupplier output) {
    return drivetrain.applyRequest(() -> {
      return targetAim
          .withVelocityX(DrivetrainConstants.MaxDrivetrainSpeed.times(-driverJoystick.getLeftY())
              .times(0.5))
          .withVelocityY(DrivetrainConstants.MaxDrivetrainSpeed.times(-driverJoystick.getLeftX())
              .times(0.5))
          .withRotationalRate(DrivetrainConstants.MaxRotationSpeed.times(output.getAsDouble()));
    });
  }
}
