// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String logDirectory = "";

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipControllerPort = 1;

  }

  public static class AutoConstants {

  }

  public static class IntakeConstants {
    public static final int kIntakeMotorId = 14;
    public static final int kBarMotorId = 15;
    public static final int kIntakeSensorId = 0;
    public static final double kExtendSetpoint = -16d;
    public static final double kRetractSetpoint = 0d;
    public static final double kPiviotP = 2;
    public static final double kPiviotI = 0d;
    public static final double kPiviotD = 8.5;
    public static final double kPiviots = 23.5;
    public static final double kStatorCurrentLimit = 80d;

    


    public static final boolean kTopIntakeMotorInverted = false;
    public static final boolean kBottomIntakeMotorInverted = false;
  }
  public static class EndeffectorConstants{

    public static final int kEndeffectorRightMotorId = 0;
    public static final int kEndeffectorLeftMotorId = 0;
    public static final int kEndeffectorSensorId = 0;

    public static final double kEndeffectorSpit = 0.6;

  }

  public static class ArmConstants {
    public static final int kArmMotorId = 30;
    public static final double kArmP = 0;
    public static final double kArmI = 0;
    public static final double kArmD = 0;
    public static final double kArmFF = 0;
    public static final GravityTypeValue kArmgravtype = GravityTypeValue.Arm_Cosine;
    public static final int kArmEncoderId = 55;
    public static final double kArmheight = 5d;
    public static final double kArmWidth = 5d;
    public static final double kArmRootX = 1d;
    public static final double kArmRootY = 2d;
    public static final double kArmRotationtoDegreeRatio = 2498d;

  }

  public static class ElevatorConstants {
    public static final int kRightElevatorMotorId = 5;
    public static final int kLeftElevatorMotorId = 5;
    public static final double kElevatorP = 50;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorFF = 1;
    public static final double kElevatorMaxCurrent = 40.0d;
    public static final double kElevatorRegStatorCurrentLimit = 40.0d;
    public static final double kElevatorHomeStatorCurrentLimit = 40.0d;
    public static final double kElevatorHomeDutyCycle = 0.5;
    public static final int kRightElevatorEncoderId = 3;
    public static final int kLeftElevatorEncoderId = 3;
    // TODO FINNNNDDDD ALLLLL THESSESEESE F******NG VALUES
    public static final double kMaxElevatorHeight = 2.27887911;
    public static final double kMinElevatorHeight = 0d;
    public static final double kRotationsToMeters = 0.15959269;
    public static final double kElevatorWeight = 24d;
    public static final double kElevatorHeight = 1d;
    public static final double kElevatorWidth = 1d;
    public static final double kRootElevatorX = .3;
    public static final double kRootElevatorY = 0d;
    public static final double kRootElevator2X = .6;
    public static final double kRootElevator2Y = 0d;
    public static final double kLigaLength = .3d;
    public static final double kElevatorGearing = 14/72d;
    public static final double kDrumRadius = Units.inchesToMeters(1);
    public static final boolean kIsLeftInvert = false;


  }

  public static class ScoringConstants {
    public static final double kArmScoringangle = 270;

    public static class L1 {
      public static final double kElevatorPos = 10d;
    }

    public static class L2 {
      public static final double kElevatorPos = 60;
    }

    public static class L3 {
      public static final double kElevatorPos = 100;
    }

    public static class L4 {
      public static final double kElevatorPos = 140;
    }

  }

  public static class SwerveConstants {
    // Both sets of gains need to be tuned to your individual robot.
    // TODO tune

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(2.66).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // TODO tune
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these
    // cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API
    // documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a
                // relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("Canivore", "./logs/example.hoot");

    public static class AutoConstants {
      public static final PIDConstants translationPID = new PIDConstants(0, 0, 0);
      public static final PIDConstants rotationPID = new PIDConstants(0, 0, 0);

      public static class AutoPoses{
        public static final Pose2d kOpenPose = new Pose2d(7.557, 7.479, new Rotation2d(0));
        public static final Pose2d kCenterPose = new Pose2d(7.557, 4.023, new Rotation2d(0));
        public static final Pose2d kProPose = new Pose2d(7.557, 0.685, new Rotation2d(0));


        
      }

      private final static MomentOfInertia kRobotMomentOfInertia = KilogramSquareMeters.of(0.01);

      private final static ModuleConfig kModuleConfig = new ModuleConfig(
          Constants.SwerveConstants.kWheelRadius,
          Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts,
          1.916,
          DCMotor.getKrakenX60Foc(1).withReduction(Constants.SwerveConstants.kDriveGearRatio),
          Amps.of(120),
          1);
      // TODO make these more efficient
      private final static Translation2d[] kModulePositions = {
          new Translation2d(Inches.of(12.125), Inches.of(12.125)),
          new Translation2d(Inches.of(12.125), Inches.of(-12.125)),
          new Translation2d(Inches.of(-12.125), Inches.of(12.125)),
          new Translation2d(Inches.of(-12.125), Inches.of(-12.125)),
      };

      // double massKG, double MOI, ModuleConfig moduleConfig, Translation2d...
      // moduleOffsets) {
      public static final RobotConfig robotConfig = new RobotConfig(
          69d,
          kRobotMomentOfInertia.baseUnitMagnitude(),
          kModuleConfig,
          kModulePositions);

      public static final String kFieldObjectName = "Auto";

      public static class AutoSpeeds {

        // Theoretical free speed (m/s) at 12 V applied output;
        // TODO tune
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.73);

        // The maximum acceleration of the robot in meters per second squared.
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // The maximum angular speed of the robot in radians per second.
        public static final double kMaxAngularSpeedRadiansPerSecond = 13.5;

        // The maximum angular acceleration of the robot in radians per second squared.
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 3;
      }
    }

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;

    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final Distance kWheelRadius = Inches.of(2);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 3;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
        .withCANBusName(kCANBus.getName())
        .withPigeon2Id(kPigeonId)
        .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
        .withDriveMotorGearRatio(kDriveGearRatio)
        .withSteerMotorGearRatio(kSteerGearRatio)
        .withCouplingGearRatio(kCoupleRatio)
        .withWheelRadius(kWheelRadius)
        .withSteerMotorGains(steerGains)
        .withDriveMotorGains(driveGains)
        .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
        .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
        .withSlipCurrent(kSlipCurrent)
        .withSpeedAt12Volts(Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts)
        .withDriveMotorType(kDriveMotorType)
        .withSteerMotorType(kSteerMotorType)
        .withFeedbackSource(kSteerFeedbackType)
        .withDriveMotorInitialConfigs(driveInitialConfigs)
        .withSteerMotorInitialConfigs(steerInitialConfigs)
        .withEncoderInitialConfigs(encoderInitialConfigs)
        .withSteerInertia(kSteerInertia)
        .withDriveInertia(kDriveInertia)
        .withSteerFrictionVoltage(kSteerFrictionVoltage)
        .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 12;
    private static final int kFrontLeftSteerMotorId = 13;
    private static final int kFrontLeftEncoderId = 11;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.01953125);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = true;

    private static final Distance kFrontLeftXPos = Inches.of(12.125);
    private static final Distance kFrontLeftYPos = Inches.of(12.125);

    // Front Right
    private static final int kFrontRightDriveMotorId = 22;
    private static final int kFrontRightSteerMotorId = 23;
    private static final int kFrontRightEncoderId = 21;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.32470703125);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = true;

    private static final Distance kFrontRightXPos = Inches.of(12.125);
    private static final Distance kFrontRightYPos = Inches.of(-12.125);

    // Back Left
    private static final int kBackLeftDriveMotorId = 32;
    private static final int kBackLeftSteerMotorId = 33;
    private static final int kBackLeftEncoderId = 31;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.231201171875);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = true;

    private static final Distance kBackLeftXPos = Inches.of(-12.125);
    private static final Distance kBackLeftYPos = Inches.of(12.125);

    // Back Right
    private static final int kBackRightDriveMotorId = 42;
    private static final int kBackRightSteerMotorId = 43;
    private static final int kBackRightEncoderId = 41;
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.0205078125);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = true;

    private static final Distance kBackRightXPos = Inches.of(-12.125);
    private static final Distance kBackRightYPos = Inches.of(-12.125);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
        .createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
        .createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted,
            kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
        .createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
        .createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted);

  }

}
