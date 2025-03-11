// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Utilities;

public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
        static Rotation2d redPerspective = Rotation2d.k180deg, bluePerspective = Rotation2d.kZero;
        boolean appliedPerspective = false;

        SwerveRequest.FieldCentric fieldCentric;
        SwerveRequest.RobotCentric robotCentric;

        RobotConfig robotConfig;

        double percentSpeed = 0.8, antiTipping;

        public Drivetrain(SwerveDrivetrainConstants drivetrainConfigs, SwerveModuleConstants<?, ?, ?>... modules) {
                super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfigs, modules);

                fieldCentric = new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.Tuner.maxSpeed * 0.1).withRotationalDeadband(Constants.Tuner.maxAngularSpeed * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                robotCentric = new SwerveRequest.RobotCentric()
                        .withDeadband(Constants.Tuner.maxSpeed * 0.1).withRotationalDeadband(Constants.Tuner.maxAngularSpeed * 0.1)
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
        }
        
        public void updateHeight(double height) {
                antiTipping = (30 - height) / 30;
        }


        public Command driveFieldCentric(ChassisSpeeds speeds) {
                return run(() -> setControl(fieldCentric
                        .withVelocityX(speeds.vxMetersPerSecond * percentSpeed * antiTipping)
                        .withVelocityY(speeds.vyMetersPerSecond * percentSpeed * antiTipping)
                        .withRotationalRate(speeds.omegaRadiansPerSecond)
                ));
        }

        public Command driveRobotCentric(ChassisSpeeds speeds) {
                return run(() -> setControl(robotCentric
                        .withVelocityX(speeds.vxMetersPerSecond * percentSpeed * antiTipping)
                        .withVelocityY(speeds.vyMetersPerSecond * percentSpeed * antiTipping)
                        .withRotationalRate(speeds.omegaRadiansPerSecond)
                ));
        }

        public Command driveToPose(Pose2d targetPose) {
                return AutoBuilder.pathfindToPose(
                        targetPose, 
                        new PathConstraints(
                                Constants.Tuner.maxSpeed, 
                                Constants.Tuner.maxAcceleration, 
                                Constants.Tuner.maxAngularSpeed, 
                                Constants.Tuner.maxAngularAcceleration
                        )
                );
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
}
