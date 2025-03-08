// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities;

public class Arm extends SubsystemBase {
        TalonFX arm;
        TalonFX rollers;
        CANrange distance;

        Timer timer;
        boolean hasAlgae;

        public enum Position {
                Stow(3.5),

                Intake_Coral(-0.1),
                Hold_Algae(21);

                public double value;

                Position(double value) {
                        this.value = value;
                }
        }

        public Arm(int armID, int rollersID, int distanceID) {
                arm = new TalonFX(armID);
                rollers = new TalonFX(rollersID);

                distance = new CANrange(distanceID);

                TalonFXConfiguration config = new TalonFXConfiguration();
                config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                config.Slot0.kP = 10;

                config.MotionMagic.MotionMagicCruiseVelocity = 40;
                config.MotionMagic.MotionMagicAcceleration = 60;
                arm.getConfigurator().apply(config);

                timer = new Timer();
                timer.start();

                hasAlgae = false;
        }

        public boolean hasCoral() {
                return distance.getIsDetected(true).getValue();
        }

        public boolean hasAlgae() {
                return hasAlgae;
        }

        public Command setPosition(Position position) {
                return new Command() {
                        public void execute() {
                                arm.setControl(new MotionMagicExpoVoltage(position.value));
                        }

                        public boolean isFinished() {
                                return Utilities.inTolerance(position.value - arm.getPosition().getValueAsDouble(), 0.2);
                        }
                };
        }

        public Command intakeCoral() {
                return new Command() {
                        public void execute() {
                                rollers.set(-0.6);
                        }

                        public boolean isFinished() {
                                return hasCoral();
                        }

                        public void end(boolean interupted) {
                                rollers.set(0);
                        }
                };
        }

        public Command intakeAlgae() {
                return new Command() {
                        public void initialize() {
                                timer.reset();
                        }

                        public void execute() {
                                rollers.set(0.5);
                        }

                        public boolean isFinished() {
                                return rollers.getVelocity().getValueAsDouble() < 1 && timer.get() > 2;
                        }

                        public void end(boolean interupted) {
                                hasAlgae = true;
                        }
                };
        }

        public Command outtakeCoral() {
                return new Command() {
                        public void execute() {
                                rollers.set(-0.5);
                        }

                        public boolean isFinished() {
                                return !hasCoral();
                        }

                        public void end(boolean interupted) {
                                rollers.set(0);
                        }
                };
        }

        public Command outtakeAlgae() {
                return new Command() {
                        public void initialize() {
                                timer.reset();
                        }

                        public void execute() {
                                rollers.set(-0.4);
                        }

                        public boolean isFinished() {
                                return timer.get() > 2;
                        }

                        public void end(boolean interupted) {
                                hasAlgae = false;
                                rollers.set(0);
                        }
                };
        }

        public Command reset() {
                return new Command() {
                        public void initialize() {
                                hasAlgae = false;
                                rollers.set(0);
                        }

                        public boolean isFinished() {
                                return true;
                        }
                };
        }

        @Override
        public void periodic() {
                if (hasAlgae) rollers.set(0.1);
        }
}
