// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Elevator;

public class Utilities {
        public enum Side {
                A(new Pose2d(), new Pose2d(), new Pose2d(), Elevator.Position.High_Algae),
                B(new Pose2d(), new Pose2d(), new Pose2d(), Elevator.Position.Low_Algae),
                C(new Pose2d(), new Pose2d(), new Pose2d(), Elevator.Position.High_Algae),
                D(new Pose2d(), new Pose2d(), new Pose2d(), Elevator.Position.Low_Algae),
                E(new Pose2d(), new Pose2d(), new Pose2d(), Elevator.Position.High_Algae),
                F(new Pose2d(), new Pose2d(), new Pose2d(), Elevator.Position.Low_Algae);

                Pose2d leftOffset, rightOffset, centerOffset;
                Elevator.Position algaeLevel;

                Side(Pose2d leftOffset, Pose2d rightOffset, Pose2d centerOffset, Elevator.Position algaeLevel) {
                        this.leftOffset = leftOffset;
                        this.rightOffset = rightOffset;
                        this.centerOffset = centerOffset;

                        this.algaeLevel = algaeLevel;
                }
        }

        public static Alliance getAlliance() {
                return DriverStation.getAlliance().get();
        }
        
        public static Pose2d addOffset(Pose2d pose, Pose2d offset) {
                return new Pose2d(
                        pose.getX() + offset.getX(),
                        pose.getY() + offset.getY(),
                        pose.getRotation()
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

        public static Side getClosestSide(Pose2d robotPose) {
                Pose2d center = getAlliance() == Alliance.Red ? Constants.Alignment.redCenter : Constants.Alignment.blueCenter;
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
