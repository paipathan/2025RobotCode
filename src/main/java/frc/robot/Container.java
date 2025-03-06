// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;

public class Container {
        Drivetrain drivetrain;
        Arm arm;
        Elevator elevator;
        CANdle status;

        SwerveRequest.FieldCentric driveRequest;

        Mode mode;
        Elevator.Position coralLevel;
        Elevator.Position algaeLevel;

        double maxSpeed = TunerConstants.maxSpeed.in(MetersPerSecond);
        double maxRotation = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        public enum Mode {
                Coral,
                Algae
        }

        public Container() {
                drivetrain = TunerConstants.createDrivetrain();
                arm = new Arm(30, 41, 32);
                elevator = new Elevator(20, 21, "drivetrain");
                
                status = new CANdle(55);

                driveRequest = new SwerveRequest.FieldCentric()
                        .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxRotation * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                mode = Mode.Coral;
                coralLevel = Elevator.Position.L2_Coral;
                algaeLevel = Elevator.Position.Low_Algae;
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

        public Command drive(double powerX, double powerY, double powerR) {
                return drivetrain.applyRequest(() -> driveRequest
                        .withVelocityX(-powerY * maxSpeed * 0.2)
                        .withVelocityY(-powerX * maxSpeed * 0.2)
                        .withRotationalRate(-powerR * maxRotation)
                );
        }

        public Command modeCoral() {
                return new Command() {
                        public void initialize() {
                                mode = Mode.Coral;
                                status.setLEDs(255, 0, 255);
                        }

                        public boolean isFinished() {
                                return true;
                        }
                };
        }

        public Command modeAlgae() {
                return new Command() {
                        public void initialize() {
                                mode = Mode.Algae;
                                status.setLEDs(0, 255, 0);
                        }

                        public boolean isFinished() {
                                return true;
                        }
                };
        }

        public Command targetLow() {
                return new Command() {
                        public void initialize() {
                                coralLevel = Elevator.Position.L2_Coral;
                                algaeLevel = Elevator.Position.Low_Algae;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                };
        }
        
        public Command targetMedium() {
                return new Command() {
                        public void initialize() {
                                coralLevel = Elevator.Position.L3_Coral;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                };
        }

        public Command targetHigh() {
                return new Command() {
                        public void initialize() {
                                coralLevel = Elevator.Position.L4_Coral;
                                algaeLevel = Elevator.Position.High_Algae;
                        }

                        public boolean isFinished() {
                                return true;
                        }
                };
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
                                        elevator.setPosition(algaeLevel),
                                        Commands.parallel(
                                                arm.setPosition(Arm.Position.Hold_Algae),
                                                arm.intakeAlgae()
                                        )
                                ),

                                () -> {
                                        return mode == Mode.Coral;
                                }
                        )
                );
        }

        public Command runOuttake() {
                return Commands.either(
                        Commands.either(
                                Commands.sequence(
                                        arm.setPosition(Arm.Position.Stow),
                                        elevator.setPosition(coralLevel),
                                        Commands.waitSeconds(0.5),
                                        arm.outtakeCoral(),
                                        elevator.setPosition(Elevator.Position.Stow)
                                ),
                                Commands.none(),

                                () -> {
                                        return arm.hasCoral();
                                }
                        ),
                        Commands.either(
                                Commands.sequence(
                                        arm.setPosition(Arm.Position.Hold_Algae),
                                        elevator.setPosition(Elevator.Position.Stow),
                                        arm.outtakeAlgae(),
                                        arm.setPosition(Arm.Position.Stow)
                                ),
                                Commands.none(),

                                () -> {
                                        return arm.hasAlgae();
                                }
                        ),

                        () -> {
                                return mode == Mode.Coral;
                        }      
                );
        }
}
