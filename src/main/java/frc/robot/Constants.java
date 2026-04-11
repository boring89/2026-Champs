// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotState;

public final class Constants {

    public static final class DrivetrainConstants {
        public static final LinearVelocity MaxDrivetrainSpeed = MetersPerSecond.of(5.29);

        public static final LinearVelocity NormalTeleOperatedSpeed = MetersPerSecond.of(4.0);
        public static final LinearVelocity MaxTeleOperatedSpeed = MetersPerSecond.of(5.29);

        public static final AngularVelocity MaxRotationSpeed = RotationsPerSecond.of(1.5);
    }

    public static final class PhotonVisionConstants {

        public static final Map<String, Transform3d> cameraTransforms = Map.of(
                "FrontRight", new Transform3d(
                        //右側
                        new Translation3d(0.3113271, -0.3113278, 0.1838034),
                        new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(-45.0))),
                "FrontLeft", new Transform3d(
                        //左側
                        new Translation3d(0.3113271, 0.3113278, 0.1838034),
                        new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(45.0))),
                    "BackRight", new Transform3d(
                        //左側
                        new Translation3d(0.1188632, -0.2648464, 0.3113339),
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-25.0), Units.degreesToRadians(-180.0))),
                    "BackLeft", new Transform3d(
                        //左側
                        new Translation3d(0.1188632, 0.2648464, 0.3113339),
                        new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-25.0), Units.degreesToRadians(-180.0))));
                    

        public static final double borderPixels = 15.0; // 拒絕貼邊緣的角點（避免畸變/遮擋）
        public static final double maxSingleTagDistanceMeters = Units.feetToMeters(10); // 單tag最遠可接受距離
        public static final double maxYawRate = 720.0;// 最大可以接受的旋轉速度
        public static final double maxZ = 0.5; //最大高度
    }
}
