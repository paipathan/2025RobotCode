package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
  TalonFX arm;
  TalonFX rollers;
  CANrange canRange;

  Timer timer;

  boolean hasAlgae = false;

  public enum OuttakeMode {
    Hard_Stop(0),
    Stow(2.4),
    L2_Coral(0),
    L3_Coral(0),
    L4_Coral(3),
    Algae(21),
    Barge(0);

    public final double pos;

    private OuttakeMode(double pos) {
        this.pos = pos;
    }
  }

  public Outtake(int armID, int rollerID, int canRangeID) {
    arm = new TalonFX(armID);
    rollers = new TalonFX(rollerID);
    canRange = new CANrange(canRangeID);

    timer = new Timer();
    timer.start();

    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    Slot0Configs armSlot0Config = new Slot0Configs();

    armSlot0Config.kP = 30;
    armConfig.MotionMagic.MotionMagicCruiseVelocity = 50;
    armConfig.MotionMagic.MotionMagicAcceleration = 80;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    armConfig.withSlot0(armSlot0Config);
    arm.getConfigurator().apply(armConfig);
  }

  @Override
  public void periodic() {
    if (hasAlgae) rollers.set(0.2);
  }



  public Command setPosition(OuttakeMode outtakeMode) {
    return new Command() {
      public void execute() {
        arm.setControl(new MotionMagicExpoVoltage(outtakeMode.pos));
      }

      public boolean isFinished() {
        return Math.abs(arm.getPosition().getValueAsDouble() - outtakeMode.pos) <= 0.5;
      }

      public void end(boolean interupted) {

      }
    };
  }


  public Command runCoralIntake() {
    return new Command() {
      public void execute() {
        rollers.set(-0.6);
      }

      public boolean isFinished() {
        return canRange.getIsDetected(true).getValue();
      }

      public void end(boolean interupted) {
        rollers.set(0);
      }
    };
  }

  public Command scoreCoral() {
    return new Command() {
      public void execute() {
        rollers.set(-0.5);
      }

      public boolean isFinished() {
        return !canRange.getIsDetected(true).getValue();
      }

      public void end(boolean interupted) {
        rollers.set(0);
      }
    };
  }


  public Command runAlgaeIntake() {
    return new Command() {
      public void initialize() {
        timer.reset();
      }

      public void execute() {
        rollers.set(0.5);
      }

      public boolean isFinished() {
        System.out.println(timer.get());
        return rollers.getVelocity().getValueAsDouble() < 1 && timer.get() > 2;
      }

      public void end(boolean interupted) {
        hasAlgae = true;
      }
    };
  }

  public Command tempReset = new Command() {
    public void initialize() {
      rollers.set(0);
      hasAlgae = false;
    }

    public boolean isFinished() {
      return true;
    }
  };


}
