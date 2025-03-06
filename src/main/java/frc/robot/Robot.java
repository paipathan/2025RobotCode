// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
        Container container;

        public Robot() {
                container = new Container();
        }

        @Override
        public void robotPeriodic() {
                CommandScheduler.getInstance().run(); 
        }

        @Override
        public void autonomousInit() {
                CommandScheduler.getInstance().cancelAll();
        }

        @Override
        public void autonomousPeriodic() {}

        @Override
        public void autonomousExit() {}

        @Override
        public void teleopInit() {
                CommandScheduler.getInstance().cancelAll();
        }

        @Override
        public void teleopPeriodic() {}

        @Override
        public void teleopExit() {}
}
