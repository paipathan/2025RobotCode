// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utility {
        public enum Side {
                A,
                B,
                C,
                D,
                E,
                F
        }

        public static Pose2d ToPose2d(Pose3d pose) {
                return new Pose2d(
                        pose.getX(),
                        pose.getY(),
                        new Rotation2d(pose.getRotation().getZ())
                );
        }

        public static Side GetSide(Alliance alliance, Pose2d robotPose) {
                Pose2d center = alliance == Alliance.Red ? new Pose2d() : new Pose2d();
                double angle = Math.atan2(robotPose.getX() - center.getX(), robotPose.getY() - center.getY()) % 360;

                if (angle > 0) return Side.C;
                if (angle > 60) return Side.D;
                if (angle > 120) return Side.E;
                if (angle > 180) return Side.F;
                if (angle > 240) return Side.A;
                if (angle > 300) return Side.B;

                return null;
        }
}
