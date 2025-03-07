// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

        Mode mode;
        Elevator.Position coralLevel;
        Elevator.Position algaeLevel;

        public enum Mode {
                Coral,
                Algae
        }

        public Container() {
                drivetrain = TunerConstants.createDrivetrain();
                arm = new Arm(30, 41, 32);
                elevator = new Elevator(20, 21, "drivetrain");
                
                status = new CANdle(55);

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

        public Command drive(double leftX, double leftY, double rightX) {
                ChassisSpeeds speeds = new ChassisSpeeds(-leftY * TunerConstants.maxSpeed, -leftX * TunerConstants.maxSpeed, -rightX * TunerConstants.maxRotation);
                return drivetrain.driveRobotCentric(speeds);
        }

        public Command stow() {
                return Commands.sequence(
                        arm.setPosition(Arm.Position.Stow),
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
                                        elevator.setPosition(algaeLevel),
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
                        Commands.either(
                                Commands.sequence(
                                        arm.setPosition(Arm.Position.Stow),
                                        elevator.setPosition(coralLevel),
                                        Commands.waitSeconds(0.5),
                                        arm.outtakeCoral()
                                ),
                                Commands.none(),

                                () -> arm.hasCoral()
                        ),
                        Commands.either(
                                Commands.sequence(
                                        arm.setPosition(Arm.Position.Hold_Algae),
                                        elevator.setPosition(Elevator.Position.Stow),
                                        arm.outtakeAlgae(),
                                        arm.setPosition(Arm.Position.Stow)
                                ),
                                Commands.none(),

                                () -> arm.hasAlgae()
                        ),

                        () -> mode == Mode.Coral
                );
        }
}
