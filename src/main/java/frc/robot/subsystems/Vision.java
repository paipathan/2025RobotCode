// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;

public class Vision extends SubsystemBase {
        AprilTagFieldLayout tagLayout;
        PhotonPoseEstimator estimator;
        
        PhotonCamera frontLeftCam, frontRightCam, backLeftCam, backRightCam;
        Transform3d frontLeftOffset, frontRightOffset, backLeftOffset, backRightOffset;

        public enum Camera {
                Front_Left,
                Front_Right,
                Back_Left,
                Back_Right
        }

        public Vision(String frontLeftID, Transform3d frontLeftOffset, String frontRightID, Transform3d frontRightOffset, String backLeftID, Transform3d backLeftOffset, String backRightID, Transform3d backRightOffset) {
                tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
                estimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, null);

                frontLeftCam = new PhotonCamera(frontLeftID);
                this.frontLeftOffset = frontLeftOffset;

                frontRightCam = new PhotonCamera(frontRightID);
                this.frontRightOffset = frontRightOffset;

                backLeftCam = new PhotonCamera(backLeftID);
                this.backLeftOffset = backLeftOffset;

                backRightCam = new PhotonCamera(backRightID);
                this.backRightOffset = backRightOffset;
        }

        PhotonCamera GetCamera(Camera camera) {
                switch (camera) {
                        case Front_Left: return frontLeftCam;
                        case Front_Right: return frontRightCam;
                        case Back_Left: return backLeftCam;
                        case Back_Right: return backRightCam;
                
                        default: return null;
                }
        }

        Transform3d GetOffset(Camera camera) {
                switch (camera) {
                        case Front_Left: return frontLeftOffset;
                        case Front_Right: return frontRightOffset;
                        case Back_Left: return backLeftOffset;
                        case Back_Right: return backRightOffset;
                
                        default: return null;
                }
        }

        public Pose2d GetPose(Camera camera) {
                estimator.setRobotToCameraTransform(GetOffset(camera));
                return Utilities.toPose2d(estimator.update(GetCamera(camera).getAllUnreadResults().get(0)).get().estimatedPose);
        }

        @Override
        public void periodic() {}
}
