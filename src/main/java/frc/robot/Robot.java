// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ButtonBoard.Action;

public class Robot extends TimedRobot {
        XboxController controller;
        ButtonBoard board;
        Container container;

        StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().
                getStructTopic("Robot Pose", Pose2d.struct).publish();

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

                if (controller.getAButtonPressed()) container.stow().schedule();

                if (controller.getLeftBumperButtonPressed()) container.runIntake().schedule();
                if (controller.getRightBumperButtonPressed()) container.runOuttake().schedule();

                if(controller.getXButtonPressed()) container.manualOuttake().schedule();
                
                publisher.set(container.getDrivetrain().getRobotPose());
                
                // container.getDrivetrain().addVisionMeasurement(container.getVision().getPose(Camera.Front), Timer.getFPGATimestamp());               
        }

        @Override
        public void teleopExit() {}
}
