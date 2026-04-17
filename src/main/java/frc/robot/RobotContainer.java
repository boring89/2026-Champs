// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.subsystems.Superstructure;
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
import frc.robot.subsystems.Mechanism.Roller.RollerIO;
import frc.robot.subsystems.Mechanism.Roller.RollerIOHardware;
import frc.robot.subsystems.Mechanism.Shooter.ShooterIO;
import frc.robot.subsystems.Mechanism.Shooter.ShooterIOHardware;
import frc.robot.subsystems.Vision.PhotonVision;
import frc.robot.utilities.RobotState.RobotState;
import frc.robot.utilities.TargetCalculator.TargetCalculator;

public class RobotContainer {

  private boolean isTest = false;

  private final CommandXboxController driverJoystick = new CommandXboxController(0);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final SwerveRequest.FieldCentric normalDrive = new SwerveRequest.FieldCentric()
      .withDeadband(DrivetrainConstants.MaxDrivetrainSpeed.times(0.1))
      .withRotationalDeadband(DrivetrainConstants.MaxRotationSpeed.times(0.1))
      .withDesaturateWheelSpeeds(true)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentric targetAim = new SwerveRequest.FieldCentric()
      .withDeadband(DrivetrainConstants.MaxDrivetrainSpeed.times(0.1))
      .withRotationalDeadband(RotationsPerSecond.of(0.01))
      .withDesaturateWheelSpeeds(true)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final PivotIO pivot = new PivotIOHardware();
  private final RollerIO roller = new RollerIOHardware();
  private final HopperIO hopper = new HopperIOHardware();
  private final FeederIO feeder = new FeederIOHardware();
  private final ShooterIO shooter = new ShooterIOHardware();
  private final HoodIO hood = new HoodIOHardware();

  private final RobotState robotState = new RobotState(drivetrain.getState());

  private final TargetCalculator targetCalculator = new TargetCalculator(robotState);
  private final Superstructure superstructure = new Superstructure(
      drivetrain, roller, hopper, feeder, pivot, shooter, hood, isTest);

  private final Telemetry logger = new Telemetry(this.drivetrain.getState());

  private final SendableChooser<Command> autoChooser;

  public final PhotonVision photonvision = new PhotonVision(drivetrain, PhotonVisionConstants.cameraTransforms);

  public RobotContainer() {
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("chooser", autoChooser);

  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          return normalDrive
              .withVelocityX(DrivetrainConstants.MaxDrivetrainSpeed.times(-driverJoystick.getLeftY())
                  .times(0.3))
              .withVelocityY(DrivetrainConstants.MaxDrivetrainSpeed.times(-driverJoystick.getLeftX())
                  .times(0.3))
              .withRotationalRate(DrivetrainConstants.MaxRotationSpeed.times(-driverJoystick.getRightX()));
        }));
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    drivetrain.registerTelemetry(logger::telemeterize);

    driverJoystick.start().and(driverJoystick.back()).onTrue(Commands.runOnce(this::toggleMode));

    

    if (isTest) {
      mechanismTest();
      sysidTest();
    } else {
      normalControls();
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void normalControls() {
    driverJoystick.a().and(() -> !isTest).onTrue(superstructure.pivotShake())
        .onFalse(superstructure.stopShake());

    driverJoystick.leftTrigger().and(() -> !isTest).onTrue(superstructure.intake())
        .onFalse(superstructure.stopIntake());
    driverJoystick.leftBumper().and(() -> !isTest).onTrue(superstructure.pivotBack());

    driverJoystick.rightTrigger().and(() -> !isTest).onTrue(superstructure.shoot())
        .onFalse(superstructure.stopShoot());
    driverJoystick.rightBumper().and(() -> !isTest).onTrue(aim(() -> superstructure.calculateDrivetrainOutput()));
  }

  public void sysidTest() {
    driverJoystick.povUp().and(driverJoystick.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverJoystick.povUp().and(driverJoystick.b()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverJoystick.povUp().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverJoystick.povUp().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  public void mechanismTest() {
    driverJoystick.a().and(() -> isTest).whileTrue(superstructure.intake());
    driverJoystick.b().and(() -> isTest).whileTrue(superstructure.hopperTest());
    driverJoystick.x().and(() -> isTest).whileTrue(superstructure.feederTest());
    driverJoystick.y().and(() -> isTest).whileTrue(superstructure.pivotTest());
    driverJoystick.povUp().and(() -> isTest).whileTrue(superstructure.shooterTest());
    driverJoystick.povDown().and(() -> isTest).whileTrue(superstructure.hoodTest());
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

  private void toggleMode() {
    isTest = !isTest;
  }
}
