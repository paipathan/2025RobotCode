// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ButtonBoard {
        CommandXboxController controller;

        public enum Action {
                Mode_Coral(1),
                Mode_Algae(2),

                Target_Low(3),
                Target_Medium(4),
                High(5),

                Align_Left(6),
                Align_Right(7),
                Align_Center(8);

                public int value;

                Action(int value) {
                        this.value = value;
                }
        }

        public ButtonBoard(int port) {
                controller = new CommandXboxController(port);
        }

        public void onTrue(Action action, Command command) {
                controller.button(action.value).onTrue(command);
        }

        public void onFalse(Action action, Command command) {
                controller.button(action.value).onFalse(command);
        }

        public void whileTrue(Action action, Command command) {
                controller.button(action.value).whileTrue(command);
        }
}
