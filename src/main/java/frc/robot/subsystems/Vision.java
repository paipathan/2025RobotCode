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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;
import frc.robot.generated.LimelightHelpers;

public class Vision extends SubsystemBase {
        String frontID;

        AprilTagFieldLayout tagLayout;
        PhotonPoseEstimator estimator;
        
        PhotonCamera leftCam, rightCam;
        Transform3d leftOffset, rightOffset;

        public enum Camera {
                Front,
                Left,
                Right
        }

        public Vision(String frontID, String leftID, String rightID, Transform3d leftOffset, Transform3d rightOffset) {
                this.frontID = frontID;

                tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
                estimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, null);

                leftCam = new PhotonCamera(leftID);
                this.leftOffset = leftOffset;

                rightCam = new PhotonCamera(rightID);
                this.rightOffset = rightOffset;
        }

        PhotonCamera getCamera(Camera camera) {
                switch (camera) {
                        case Left: return leftCam;
                        case Right: return rightCam;
                
                        default: return null;
                }
        }

        Transform3d getOffset(Camera camera) {
                switch (camera) {
                        case Left: return leftOffset;
                        case Right: return rightOffset;
                
                        default: return null;
                }
        }

        public Pose2d getPose(Camera camera) {
                if (camera == Camera.Front) {
                       return Utilities.getAlliance() == Alliance.Red ? LimelightHelpers.getRedPose(frontID) : LimelightHelpers.getBluePose(frontID);
                }

                estimator.setRobotToCameraTransform(getOffset(camera));
                return Utilities.toPose2d(estimator.update(getCamera(camera).getAllUnreadResults().get(0)).get().estimatedPose);
        }

        @Override
        public void periodic() {}
}
