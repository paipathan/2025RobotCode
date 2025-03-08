// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.Drivetrain;

public class RobotConstants {

        //--ARM--//
        public static final int armID = 30;
        public static final int rollerID = 41;
        public static final int distanceID = 32;

        //--ELEVATOR--//
        public static final int leftElevatorID = 20;
        public static final int rightElevatorID = 21;
        public static final String elevatorCANBus  = "drivetrain";
        public static final int statusID = 55;

        //--DRIVETRAIN--//
        static Slot0Configs driveGains = new Slot0Configs()
                .withKP(0.1).withKI(0).withKD(0)
                .withKS(0).withKV(0.124);

        static Slot0Configs steerGains = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.5)
                .withKS(0.1).withKV(2.66).withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        static ClosedLoopOutputType driveOutputType = ClosedLoopOutputType.Voltage;
        static ClosedLoopOutputType steerOutputType = ClosedLoopOutputType.Voltage;

        static DriveMotorArrangement driveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        static SteerMotorArrangement steerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        static SteerFeedbackType feedbackType = SteerFeedbackType.FusedCANcoder;

        static Current slipCurrent = Amps.of(120.0);

        static TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        static TalonFXConfiguration steerConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );

        static CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
        static Pigeon2Configuration pigeonConfigs = null;

        public static CANBus can = new CANBus("drivetrain");
        public static double maxSpeed = 4.73;
        public static double maxRotation = 4.71;
        public static double maxAcceleration = 7.3;
        public static double maxAngularAcceleration = 2171;

        static double coupleRatio = 3.5714285714285716;
        static double driveRatio = 6.746031746031747;
        static double steerRatio = 21.428571428571427;

        static Distance wheelRadius = Inches.of(2);

        static Voltage driveFriction = Volts.of(0.2);
        static Voltage steerFriction = Volts.of(0.2);

        static SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(driveRatio)
                .withSteerMotorGearRatio(steerRatio)
                .withCouplingGearRatio(coupleRatio)
                .withWheelRadius(wheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerOutputType)
                .withDriveMotorClosedLoopOutput(driveOutputType)
                .withSlipCurrent(slipCurrent)
                .withSpeedAt12Volts(maxSpeed)
                .withDriveMotorType(driveMotorType)
                .withSteerMotorType(steerMotorType)
                .withFeedbackSource(feedbackType)
                .withDriveMotorInitialConfigs(driveConfigs)
                .withSteerMotorInitialConfigs(steerConfigs)
                .withEncoderInitialConfigs(encoderConfigs)
                .withSteerFrictionVoltage(steerFriction)
                .withDriveFrictionVoltage(driveFriction);

        static int gyroID = 13;

        static int frontLeftDriveID = 5, frontLeftSteerID = 6, frontLeftEncoderID = 52;
        static Angle frontLeftOffset = Rotations.of(0.036865234375);
        static Distance frontLeftX = Inches.of(12.625), frontLeftY = Inches.of(11.3125);

        static int frontRightDriveID = 3, frontRightSteerID = 4, frontRightEncoderID = 51;
        static Angle frontRightOffset = Rotations.of(-0.439208984375);
        static Distance frontRightX = Inches.of(12.625), frontRightY = Inches.of(-11.3125);

        static int backLeftDriveID = 7, backLeftSteerID = 8, backLeftEncoderID = 54;
        static Angle backLeftOffset = Rotations.of(-0.34912109375);
        static Distance backLeftX = Inches.of(-12.625), backLeftY = Inches.of(11.3125);

        static int backRightDriveID = 1, backRightSteerID = 2, backRightEncoderID = 53;
        static Angle backRightOffset = Rotations.of(0.36181640625);
        static Distance backRightX = Inches.of(-12.625), backRightY = Inches.of(-11.3125);

        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeftConfigs =
        ConstantCreator.createModuleConstants(
                frontLeftSteerID, frontLeftDriveID, frontLeftEncoderID, frontLeftOffset,
                frontLeftX, frontLeftY, false, true, false
        );
        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRightConfigs =
        ConstantCreator.createModuleConstants(
                frontRightSteerID, frontRightDriveID, frontRightEncoderID, frontRightOffset,
                frontRightX, frontRightY, true, true, false
        );
        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeftConfigs =
        ConstantCreator.createModuleConstants(
                backLeftSteerID, backLeftDriveID, backLeftEncoderID, backLeftOffset,
                backLeftX, backLeftY, false, true, false
        );
        public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRightConfigs =
        ConstantCreator.createModuleConstants(
                backRightSteerID, backRightDriveID, backRightEncoderID, backRightOffset,
                backRightX, backRightY, true, true, false
        );

        public static SwerveDrivetrainConstants drivetrainConfigs = new SwerveDrivetrainConstants()
        .withCANBusName(can.getName())
        .withPigeon2Id(gyroID)
        .withPigeon2Configs(pigeonConfigs);

        public static Drivetrain createDrivetrain() {
                return new Drivetrain(
                        drivetrainConfigs, frontLeftConfigs, frontRightConfigs, backLeftConfigs, backRightConfigs
                );
        }

        public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
                public TunerSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConfigs, SwerveModuleConstants<?, ?, ?>... modules) {
                        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConfigs, modules);
                }
        }
}
