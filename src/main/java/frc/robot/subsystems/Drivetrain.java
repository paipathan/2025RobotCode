// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Utilities;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
        static Rotation2d bluePerspective = Rotation2d.kZero;
        static Rotation2d redPerspective = Rotation2d.k180deg;
        boolean appliedPerspective = false;

        SwerveRequest.FieldCentric fieldCentric;
        SwerveRequest.RobotCentric robotCentric;

        RobotConfig robotConfig;

        double percentSpeed = 0.2;

        public Drivetrain(SwerveDrivetrainConstants drivetrainConfigs, SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConfigs, modules);

                fieldCentric = new SwerveRequest.FieldCentric()
                        .withDeadband(TunerConstants.maxSpeed * 0.1).withRotationalDeadband(TunerConstants.maxRotation * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                robotCentric = new SwerveRequest.RobotCentric()
                        .withDeadband(TunerConstants.maxSpeed * 0.1).withRotationalDeadband(TunerConstants.maxRotation * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                try {
                        robotConfig = RobotConfig.fromGUISettings();
                }
                
                catch (Exception error) {
                        error.printStackTrace();
                }

                AutoBuilder.configure(
                        () -> getState().Pose,
                        this::resetPose,
                        () -> getState().Speeds,
                        this::driveRobotCentric,
                        new PPHolonomicDriveController(
                                new PIDConstants(0, 0, 0),
                                new PIDConstants(0, 0, 0)
                        ),
                        robotConfig,
                        () -> Utilities.getAlliance() == Alliance.Red,
                        this
                );

                getPigeon2().reset();
                seedFieldCentric();
        }

        public Command driveFieldCentric(ChassisSpeeds speeds) {
                return run(() -> setControl(fieldCentric
                        .withVelocityX(speeds.vxMetersPerSecond * percentSpeed)
                        .withVelocityY(speeds.vyMetersPerSecond * percentSpeed)
                        .withRotationalRate(speeds.omegaRadiansPerSecond)
                ));
        }

        public Command driveRobotCentric(ChassisSpeeds speeds) {
                return run(() -> setControl(robotCentric
                        .withVelocityX(speeds.vxMetersPerSecond * percentSpeed)
                        .withVelocityY(speeds.vyMetersPerSecond * percentSpeed)
                        .withRotationalRate(speeds.omegaRadiansPerSecond)
                ));
        }

        @Override
        public void periodic() {
                if (!appliedPerspective || DriverStation.isDisabled()) {
                        DriverStation.getAlliance().ifPresent(allianceColor -> {
                                setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red ? redPerspective : bluePerspective
                                );

                                appliedPerspective = true;
                        } );
                }
        }

        @Override
        public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
                super.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestamp));
        }
}
