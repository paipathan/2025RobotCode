// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ButtonBoard {
        CommandXboxController controller;

        public enum Action {
                Outtake_L2(0),
                Outtake_L3(1),
                Outtake_L4(2),

                Intake_Reef(3),
                Outtake_Processor(4),

                Align_Left(5),
                Align_Right(6),
                Align_Center(7);

                public int value;

                Action(int value) {
                        this.value = value;
                }
        }

        public ButtonBoard(int controllerID) {
                controller = new CommandXboxController(controllerID);
        }

        public void buttonPressed(Action action, Command command) {
                controller.button(action.value).onTrue(command);
        }

        public void buttonReleased(Action action, Command command) {
                controller.button(action.value).onFalse(command);
        }

        public void button(Action action, Command command) {
                controller.button(action.value).whileTrue(command);
        }
}
