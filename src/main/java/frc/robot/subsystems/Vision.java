// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Utilities;

public class Vision extends SubsystemBase {
        String frontID;

        public Vision(String frontID) {
                this.frontID = frontID;
        }

        public Pose2d getPoseEstimate() {
                return Utilities.getAlliance() == Alliance.Red ? LimelightHelpers.getRedPoseEstimate(frontID) : LimelightHelpers.getBluePoseEstimate(frontID);
        }

        public double getTX() {
                return Constants.Vision.table.getEntry("tx").getDouble(0.0);
        }       

        public double getTY() {
                return Constants.Vision.table.getEntry("ty").getDouble(0.0);   
        }

        public double getTagDistance() {
                return (6-4)/Math.tan(30+getTY()); // Change 5 (height of camera off ground)
        }
        
        @Override
        public void periodic() {}
}
