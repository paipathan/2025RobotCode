// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.nio.file.attribute.PosixFilePermission;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Elevator.Position;
import frc.robot.subsystems.Outtake.OuttakeMode;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public Outtake outtake;
    public Elevator elevator;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    private final CommandXboxController driver = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        outtake = new Outtake(30, 41, 32);
        elevator = new Elevator(20, 21, "drivetrain");
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        // ));

        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));




        driver.leftBumper().onTrue( // INTAKE
                outtake.setPosition(OuttakeMode.Stow).andThen(
                    elevator.setPosition(Position.Stow).andThen(
                    outtake.setPosition(OuttakeMode.Hard_Stop).andThen(
                    outtake.runCoralIntake().andThen(
                    outtake.setPosition(OuttakeMode.Stow)
                )))
        ));

        driver.rightBumper().onTrue( // SCORE
            outtake.scoreCoral()
        );



        // driver.a().onTrue // PREP L2
        // (outtake.setPosition(OuttakeMode.Stow).andThen(
        //     elevator.setPosition(Position.L2_Coral).andThen(
        //         outtake.setPosition(OuttakeMode.L2_Coral)
        //     )
        // ));


        // driver.x().onTrue // PREP L3
        // (outtake.setPosition(OuttakeMode.Stow).andThen(
        //     elevator.setPosition(Position.L3_Coral).andThen(
        //         outtake.setPosition(OuttakeMode.L2_Coral)
        //     )
        // ));

        // driver.b().onTrue // PREP L4
        // (outtake.setPosition(OuttakeMode.Stow).andThen(
        //     elevator.setPosition(Position.L4_Coral).andThen(
        //         outtake.setPosition(OuttakeMode.L2_Coral)
        //     )
        // ));

        driver.a().onTrue( // CLEAN L2 ALGAE
                outtake.setPosition(OuttakeMode.Stow).andThen(elevator.setPosition(Position.Low_Algae).andThen(
                    outtake.setPosition(OuttakeMode.Algae).andThen(
                    outtake.runAlgaeIntake()
                ))
        ));

        driver.b().onTrue( // SCORE PROCESSOR
            outtake.setPosition(OuttakeMode.Algae).andThen(
                elevator.setPosition(Position.Stow).andThen(
                outtake.scoreProcessor()))
        );

        driver.x().onTrue( // RESET ROBOT
            outtake.setPosition(OuttakeMode.Stow).andThen(
                elevator.setPosition(Position.Stow)));


        





        
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
