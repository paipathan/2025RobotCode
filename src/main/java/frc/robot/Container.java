// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm;

public class Container {
        Drivetrain drivetrain;
        Arm arm;
        Elevator elevator;
        Vision vision;
        CANdle status;

        Mode mode;
        Elevator.Position coralLevel;
        Elevator.Position algaeLevel;
        Arm.Position coralScorePosition;

        public enum Mode {
                Coral,
                Algae
        }

        public Container() {
                drivetrain = RobotConstants.createDrivetrain();
                arm = new Arm(RobotConstants.armID, RobotConstants.rollerID, RobotConstants.distanceID);
                elevator = new Elevator(RobotConstants.leftElevatorID, RobotConstants.rightElevatorID, RobotConstants.elevatorCANBus);
                vision =  new Vision("limelight-front", null, null, null, null);
        
                status = new CANdle(RobotConstants.statusID);

                mode = Mode.Coral;
                coralLevel = Elevator.Position.L2_Coral;
                algaeLevel = Elevator.Position.Low_Algae;
                coralScorePosition = Arm.Position.Stow;

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

        public void modeCoral() {
                mode = Mode.Coral;
                status.setLEDs(255, 0, 255);
        }

        public void modeAlgae() {
                mode = Mode.Algae;
                status.setLEDs(0, 255, 0);
        }

        public void targetLow() {
                coralLevel = Elevator.Position.L2_Coral;
                algaeLevel = Elevator.Position.Low_Algae;
                coralScorePosition = Arm.Position.Stow;
        }
        
        public void targetMedium() {
                coralLevel = Elevator.Position.L3_Coral;
                coralScorePosition = Arm.Position.Stow;
        }

        public void targetHigh() {
                coralLevel = Elevator.Position.L4_Coral;
                algaeLevel = Elevator.Position.High_Algae;
                coralScorePosition = Arm.Position.L4_Coral;
        }

        public Command drive(double leftX, double leftY, double rightX) {
                ChassisSpeeds speeds = new ChassisSpeeds(-leftY * RobotConstants.maxSpeed, -leftX * RobotConstants.maxSpeed, -rightX * RobotConstants.maxRotation);
                return drivetrain.driveFieldCentric(speeds);
        }

        public Command stow() {
                return Commands.sequence(
                        Commands.parallel(
                                arm.setPosition(Arm.Position.Stow),
                                arm.reset()
                        ),
                        elevator.setPosition(Elevator.Position.Stow)
                );
        }


        public Command manualOuttake() {
                return Commands.either(
                                arm.outtakeCoral(), 
                                arm.outtakeAlgae(), 
                        () -> mode == Mode.Coral);
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
                                        arm.setPosition(coralScorePosition)
                                ),
                                Commands.none(),

                                () -> arm.hasCoral()
                        ),
                        Commands.either(
                                Commands.sequence(
                                        arm.setPosition(Arm.Position.Hold_Algae),
                                        elevator.setPosition(Elevator.Position.Stow)
                                ),
                                Commands.none(),

                                () -> arm.hasAlgae()
                        ),

                        () -> mode == Mode.Coral
                );
        }
}
