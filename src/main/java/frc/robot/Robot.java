// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ButtonBoard.Action;

public class Robot extends TimedRobot {
        XboxController controller;
        ButtonBoard board;

        Container container;

        public Robot() {
                controller = new XboxController(0);
                board = new ButtonBoard(1);

                container = new Container();

                CommandScheduler.getInstance().cancelAll();
        }

        @Override
        public void robotPeriodic() {
                CommandScheduler.getInstance().run();
        }

        @Override
        public void autonomousInit() {}

        @Override
        public void autonomousPeriodic() {}

        @Override
        public void autonomousExit() {}

        @Override
        public void teleopInit() {}

        @Override
        public void teleopPeriodic() {
                container.drive(controller.getLeftX(), controller.getLeftY(), controller.getRightX()).schedule();
                container.getDrivetrain().updateHeight(container.getElevator().getPosition());

                if (board.getButtonPressed(Action.Mode_Coral)) container.modeCoral();
                if (board.getButtonPressed(Action.Mode_Algae)) container.modeAlgae();

                if (board.getButtonPressed(Action.Target_Low)) container.targetLow();
                if (board.getButtonPressed(Action.Target_Medium)) container.targetMedium();
                if (board.getButtonPressed(Action.Target_High)) container.targetHigh();

                if (controller.getAButtonPressed()) container.cancel().schedule();

                if (controller.getLeftBumperButtonPressed()) container.runIntake().schedule();
                if (controller.getRightBumperButtonPressed()) container.runOuttake().schedule();
                
                container.getDrivetrain().addVisionMeasurement(null, Timer.getFPGATimestamp());
        }

        @Override
        public void teleopExit() {}
}
