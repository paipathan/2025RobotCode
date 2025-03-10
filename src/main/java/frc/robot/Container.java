// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;

public class Container {
        Drivetrain drivetrain;
        Arm arm;
        Elevator elevator;

        CANdle lights;

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

                lights = new CANdle(Constants.lightsID);

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
                lights.setLEDs(255, 0, 255);
        }

        public void modeAlgae() {
                mode = Mode.Algae;
                lights.setLEDs(0, 255, 0);
        }

        public Command drive(double leftX, double leftY, double rightX) {
                ChassisSpeeds speeds = new ChassisSpeeds(-leftY * Constants.Tuner.maxSpeed, -leftX * Constants.Tuner.maxSpeed, -rightX * Constants.Tuner.maxAngularSpeed);
                return drivetrain.driveFieldCentric(speeds);
        }

        public Command stow() {
                return Commands.sequence(
                        Commands.either(
                                arm.setPosition(Arm.Position.Stow),
                                arm.setPosition(Arm.Position.Hold_Algae),
                                
                                () -> !arm.hasAlgae()
                        ),
                        elevator.setPosition(Elevator.Position.Stow)
                );
        }

        public Command targetLow() {
                return Commands.sequence(
                        arm.setPosition(Arm.Position.Stow),
                        Commands.either(
                                elevator.setPosition(Elevator.Position.L2_Coral),
                                elevator.setPosition(Elevator.Position.Low_Algae),

                                () -> mode == Mode.Coral
                        )
                );
        }

        public Command targetMedium() {
                return Commands.sequence(
                        arm.setPosition(Arm.Position.Stow),
                        Commands.either(
                                elevator.setPosition(Elevator.Position.L3_Coral),
                                Commands.none(),

                                () -> mode == Mode.Coral
                        )
                );
        }

        public Command targetHigh() {
                return Commands.sequence(
                        arm.setPosition(Arm.Position.Stow),
                        Commands.either(
                                elevator.setPosition(Elevator.Position.L4_Coral),
                                elevator.setPosition(Elevator.Position.High_Algae),

                                () -> mode == Mode.Coral
                        )
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
                                                arm.intakeAlgae()
                                        )
                                ),

                                () -> mode == Mode.Coral
                        )
                );
        }

        public Command runOuttake() {
                return Commands.either(
                        arm.outtakeCoral(),
                        arm.outtakeAlgae(),

                        () -> mode == Mode.Coral
                );
        }
}
