// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ButtonBoard.Action;

public class Robot extends TimedRobot {
        CommandXboxController controller;
        ButtonBoard board;

        Container container;

        public Robot() {
                controller = new CommandXboxController(0);
                board = new ButtonBoard(1);

                container = new Container();

                CommandScheduler.getInstance().cancelAll();
        }

        @Override
        public void robotPeriodic() {
                CommandScheduler.getInstance().run();

                SmartDashboard.putString("Mode", container.getMode().toString());

                SmartDashboard.putString("Coral Level", container.getCoralLevel().toString());
                SmartDashboard.putString("Algae Level", container.getAlgaeLevel().toString());

                SmartDashboard.updateValues();
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

                if (board.getButton(Action.Mode_Coral)) container.modeCoral().schedule();
                if (board.getButton(Action.Mode_Algae)) container.modeAlgae().schedule();

                if (board.getButton(Action.Target_Low)) container.targetLow().schedule();
                if (board.getButton(Action.Target_Medium)) container.targetMedium().schedule();
                if (board.getButton(Action.Target_High)) container.targetHigh().schedule();

                if (controller.leftBumper().getAsBoolean()) container.runIntake().schedule();
                if (controller.rightBumper().getAsBoolean()) container.runOuttake().schedule();
        }

        @Override
        public void teleopExit() {}
}
