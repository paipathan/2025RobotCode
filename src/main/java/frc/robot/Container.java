// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ButtonBoard.Action;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;

public class Container {
        CommandXboxController driverController;
        ButtonBoard operatorBoard;

        Drivetrain drivetrain;
        Arm arm;
        Elevator elevator;

        SwerveRequest.FieldCentric driveRequest;

        Mode mode;
        Elevator.Position targetLevel;

        double maxSpeed = TunerConstants.maxSpeed.in(MetersPerSecond);
        double maxRotation = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        public enum Mode {
                Coral,
                Algae
        }

        public Container() {
                driverController = new CommandXboxController(0);
                operatorBoard = new ButtonBoard(1);

                drivetrain = TunerConstants.createDrivetrain();
                arm = new Arm(30, 41, 32);
                elevator = new Elevator(20, 21, "drivetrain");

                driveRequest = new SwerveRequest.FieldCentric()
                        .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxRotation * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                mode = Mode.Coral;
                targetLevel = Elevator.Position.L2_Coral;

                configureBindings();
        }

        void configureBindings() {
                drivetrain.setDefaultCommand(
                        drivetrain.applyRequest(() -> driveRequest
                                .withVelocityX(-driverController.getLeftY() * maxSpeed * 0.2)
                                .withVelocityY(-driverController.getLeftX() * maxSpeed * 0.2)
                                .withRotationalRate(-driverController.getRightX() * maxRotation)
                        )
                );
                
                // Set Mode
                operatorBoard.onTrue(Action.Mode_Coral, new Command() {
                        public void initialize() {
                                mode = Mode.Coral;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                });

                operatorBoard.onTrue(Action.Mode_Algae, new Command() {
                        public void initialize() {
                                mode = Mode.Algae;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                });

                // Set Target
                operatorBoard.onTrue(Action.Target_Low, new Command() {
                        public void initialize() {
                                targetLevel = mode == Mode.Coral ? Elevator.Position.L2_Coral : Elevator.Position.Low_Algae;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                });

                operatorBoard.onTrue(Action.Target_Medium, new Command() {
                        public void initialize() {
                                if (mode == Mode.Coral) targetLevel = Elevator.Position.L3_Coral;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                });

                operatorBoard.onTrue(Action.High, new Command() {
                        public void initialize() {
                                targetLevel = mode == Mode.Coral ? Elevator.Position.L4_Coral : Elevator.Position.High_Algae;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                });

                driverController.a().onTrue(
                        arm.reset()
                );

                // Intake
                driverController.leftBumper().onTrue(Commands.sequence(
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
                                        elevator.setPosition(targetLevel),
                                        Commands.parallel(
                                                arm.setPosition(Arm.Position.Hold_Algae),
                                                arm.intakeAlgae()
                                        )
                                ),

                                () -> {
                                        return mode == Mode.Coral;
                                }
                        )
                ));

                // Outtake
                driverController.rightBumper().onTrue(Commands.either(
                        Commands.sequence(
                                arm.setPosition(Arm.Position.Stow),
                                elevator.setPosition(targetLevel),
                                arm.outtakeCoral(),
                                elevator.setPosition(Elevator.Position.Stow)
                        ),

                        Commands.sequence(
                                arm.setPosition(Arm.Position.Hold_Algae),
                                arm.outtakeAlgae(),
                                arm.setPosition(Arm.Position.Stow)
                        ),

                        () -> {
                                return mode == Mode.Coral;
                        }      
                ));
        }
}
