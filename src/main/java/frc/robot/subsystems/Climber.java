// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities;

public class Climber extends SubsystemBase {

    public enum Position {
    STOW(0),
    DEPLOY(0);

    public double value;

    Position(double value) {
            this.value = value;
    }
  }

  public TalonFX climber;

  public Climber() {

    climber = new TalonFX(Constants.Climber.climberID);

    TalonFXConfiguration climberConfigs = new TalonFXConfiguration();

    climberConfigs.Slot0.kP = 0;

  }

  public Command setPosition(Position position) {
    return new Command() {
      public void execute() {
        climber.setControl(new MotionMagicExpoVoltage(position.value));
      }

      public boolean isFinished() {
        return Utilities.inTolerance(position.value - climber.getPosition().getValueAsDouble(), 0.5);
      }
    };
  }
}
