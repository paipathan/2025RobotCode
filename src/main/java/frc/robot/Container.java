// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm;

public class Container {
        Drivetrain drivetrain;
        Arm arm;
        Elevator elevator;
        Vision vision;

        Mode mode;
        Elevator.Position coralLevel;
        Elevator.Position algaeLevel;

        public enum Mode {
                Coral,
                Algae
        }

        public Container() {
                drivetrain = new Drivetrain(Constants.Tuner.drivetrainConfigs, Constants.Tuner.frontLeftConfigs, Constants.Tuner.frontRightConfigs, Constants.Tuner.backLeftConfigs, Constants.Tuner.backRightConfigs);
                arm = new Arm(Constants.Arm.pivotID, Constants.Arm.rollersID, Constants.Arm.distanceID);
                elevator = new Elevator(Constants.Elevator.leftID, Constants.Elevator.rightID, Constants.canivoreID);
                vision = new Vision(Constants.Vision.frontID);

                mode = Mode.Coral;
                coralLevel = Elevator.Position.L2_Coral;
                algaeLevel = Elevator.Position.Low_Algae;
        }

        public Drivetrain getDrivetrain() {
                return drivetrain;
        }

        public Arm getArm() {
                return arm;
        }

        public Elevator getElevator() {
                return elevator;
        }

        public Vision getVision() {
                return vision;
        }

        public Mode getMode() {
                return mode;
        }

        public Elevator.Position getCoralLevel() {
                return coralLevel;
        }

        public Elevator.Position getAlgaeLevel() {
                return algaeLevel;
        }

        public Pose2d getRobotPose() {
                return drivetrain.getState().Pose;
        }

        public void modeCoral() {
                mode = Mode.Coral;
        }

        public void modeAlgae() {
                mode = Mode.Algae;
        }

        public void targetLow() {
                coralLevel = Elevator.Position.L2_Coral;
                algaeLevel = Elevator.Position.Low_Algae;
        }

        public void targetMedium() {
                coralLevel = Elevator.Position.L3_Coral;
        }

        public void targetHigh() {
                coralLevel = Elevator.Position.L4_Coral;
                algaeLevel = Elevator.Position.High_Algae;
        }

        public Command drive(double leftX, double leftY, double rightX) {
                ChassisSpeeds speeds = new ChassisSpeeds(-leftY * Constants.Tuner.maxSpeed, -leftX * Constants.Tuner.maxSpeed, -rightX * Constants.Tuner.maxAngularSpeed);
                return drivetrain.driveFieldCentric(speeds);
        }

        public Command stow() {
                return Commands.sequence(
                        Commands.either(
                                arm.setPosition(Arm.Position.Hold_Algae),
                                arm.setPosition(Arm.Position.Stow),
                                
                                () -> arm.hasAlgae()
                        ),
                        elevator.setPosition(Elevator.Position.Stow)
                );
        }

        public Command runIntake() {
                return Commands.sequence(
                        arm.setPosition(Arm.Position.Stow),
                        Commands.either(
                                Commands.sequence(
                                        elevator.setPosition(Elevator.Position.Stow),
                                        Commands.parallel(
                                                arm.setPosition(Arm.Position.Intake_Coral),
                                                arm.intakeCoral()
                                        ),
                                        arm.setPosition(Arm.Position.Stow)
                                ),
                                Commands.sequence(
                                        Commands.parallel(
                                                arm.setPosition(Arm.Position.Hold_Algae),
                                                elevator.setPosition(algaeLevel),
                                                arm.intakeAlgae()
                                        )
                                ),

                                () -> mode == Mode.Coral
                        )
                );
        }

        public Command runOuttake() {
                return Commands.either(
                        Commands.either(
                                arm.outtakeCoral(),
                                Commands.sequence(
                                        arm.setPosition(Arm.Position.Stow),
                                        elevator.setPosition(coralLevel)
                                ),
                                
                                () -> Utilities.inTolerance(coralLevel.value - elevator.getPosition(), 0.4)
                        ),
                        arm.outtakeAlgae(),

                        () -> mode == Mode.Coral
                );
        }
}
