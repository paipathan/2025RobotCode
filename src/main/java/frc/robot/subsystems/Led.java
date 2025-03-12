// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Led extends SubsystemBase {
  Spark led;

  public enum Status {
    FLASHING_RED(0.07),
    FLASHING_GREEN(0.27);

    public double value;

    Status(double value) {
            this.value = value;
    }
}

  public Led() {
    led = new Spark(Constants.Led.ledID);
  }

  public Command setStatus(Status status) {
    return new Command() {
      public void execute() {led.set(status.value);}
      public boolean isFinished() {return true;} 
    };
  }


  @Override
  public void periodic() {

  }
}
