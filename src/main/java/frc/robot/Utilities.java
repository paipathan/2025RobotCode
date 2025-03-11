// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utilities {
        public static Alliance getAlliance() {
                return DriverStation.getAlliance().get();
        }

        public static boolean inTolerance(double error, double tolerance) {
                return Math.abs(error) < tolerance;
        }

        public static int getClosestSide(Pose2d[] centers, Pose2d robotPose) {
                int closestSide = 0;
                double leastDist = Double.MAX_VALUE;

                for (int side = 0; side < centers.length; side++) {
                        Pose2d center = centers[side];

                        double distance = Math.hypot(center.getX() - robotPose.getX(), center.getY() - robotPose.getY());

                        if (distance < leastDist) {
                                closestSide = side;
                                leastDist = distance;
                        }
                }

                return closestSide;
        }
}
