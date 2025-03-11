// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ButtonBoard.Action;
import frc.robot.Utilities.Side;

public class Robot extends TimedRobot {
        XboxController controller;
        ButtonBoard board;

        Container container;

        StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().
                getStructTopic("Robot Pose", Pose2d.struct).publish();

        public Robot() {
                controller = new XboxController(Constants.controllerID);
                board = new ButtonBoard(Constants.boardID);

                container = new Container();

                CommandScheduler.getInstance().cancelAll();

                SmartDashboard.putNumber("tx", container.getVision().getTX());
                SmartDashboard.putNumber("ty", container.getVision().getTY());
                SmartDashboard.putNumber("Tag Distance", container.getVision().getTagDistance());


        }

        @Override
        public void robotPeriodic() {
                CommandScheduler.getInstance().run();

                container.getDrivetrain().updateHeight(container.getElevator().getPosition());

                container.getDrivetrain().addVisionMeasurement(container.getVision().getPoseEstimate(), Timer.getFPGATimestamp());
                publisher.set(container.getRobotPose());

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
                if (controller.getXButtonPressed()) container.getDrivetrain().seedFieldCentric();

                if (board.getButtonPressed(Action.Mode_Coral)) container.modeCoral();
                if (board.getButtonPressed(Action.Mode_Algae)) container.modeAlgae();

                if (controller.getAButtonPressed()) container.stow().schedule();
                if (controller.getXButtonPressed()) container.getDrivetrain().seedFieldCentric();
                
                if (board.getButtonPressed(Action.Target_Low)) container.targetLow().schedule();
                if (board.getButtonPressed(Action.Target_Medium)) container.targetMedium().schedule();
                if (board.getButtonPressed(Action.Target_High)) container.targetHigh().schedule();

                Pose2d center = Utilities.getAlliance() == Alliance.Red ? Constants.Alignment.redCenter : Constants.Alignment.blueCenter;
                Side side = Utilities.getClosestSide(container.getRobotPose());
                
                if (board.getButtonPressed(Action.Align_Left)) container.getDrivetrain().driveToPose(Utilities.addOffset(center, side.leftOffset));
                if (board.getButtonPressed(Action.Align_Right)) container.getDrivetrain().driveToPose(Utilities.addOffset(center, side.rightOffset));
                if (board.getButtonPressed(Action.Align_Center)) container.getDrivetrain().driveToPose(Utilities.addOffset(center, side.centerOffset));

                if (controller.getLeftBumperButtonPressed()) container.runIntake().schedule();
                if (controller.getRightBumperButtonPressed()) container.runOuttake().schedule();   
                
        }

        @Override
        public void teleopExit() {}
}
