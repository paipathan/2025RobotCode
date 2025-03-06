// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
        static Rotation2d bluePerspective = Rotation2d.kZero;
        static Rotation2d redPerspective = Rotation2d.k180deg;
        boolean appliedPerspective = false;

        public Drivetrain(SwerveDrivetrainConstants drivetrainConfigs, SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConfigs, modules);
                seedFieldCentric();
        }

        public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
                return run(() -> this.setControl(requestSupplier.get()));
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
