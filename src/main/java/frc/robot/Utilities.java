// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utilities {
        public enum Side {
                A,
                B,
                C,
                D,
                E,
                F
        }

        public static Pose2d toPose2d(Pose3d pose) {
                return new Pose2d(
                        pose.getX(),
                        pose.getY(),
                        new Rotation2d(pose.getRotation().getZ())
                );
        }

        public static boolean inTolerance(double error, double tolerance) {
                return Math.abs(error) < tolerance;
        }

        public static Pose2d getClosestPose(Pose2d[] poses, Pose2d robotPose) {
                Pose2d closestPose = new Pose2d();
                double leastDist = Double.MAX_VALUE;

                for (Pose2d pose : poses) {
                        double distance = Math.hypot(pose.getX() - robotPose.getX(), pose.getY() - robotPose.getY());

                        if (distance < leastDist) {
                                closestPose = pose;
                                leastDist = distance;
                        }
                }

                return closestPose;
        }

        public static Side getClosestSide(Alliance alliance, Pose2d robotPose) {
                Pose2d center = alliance == Alliance.Red ? new Pose2d() : new Pose2d();
                double angle = Math.atan2(robotPose.getX() - center.getX(), robotPose.getY() - center.getY()) % 360;

                if (angle > 300) return Side.B;
                if (angle > 240) return Side.A;
                if (angle > 180) return Side.F;
                if (angle > 120) return Side.E;
                if (angle > 60) return Side.D;
                if (angle > 0) return Side.C;

                return null;
        }
}
