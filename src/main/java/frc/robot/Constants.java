// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Vector;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
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

  public static class ClimberConstants {

    public static final int kClimberMotorId = 46;
    public static final int kServoId = 9;
    public static final double kGearboxRotationsToMechanismMeters = 1d;

    // Deploy Constants
    public static final double kMaxExtension = 125;
       public static final double kExtensionDutyCycle = 0.9;

    // Reel Constants
    public static final double kReelDutyCycle = -0.9;
    public static final double kReelSafe = 5;

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipControllerPort = 1;

    public static final double kDriverDeadband = Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
        .baseUnitMagnitude() * 0.07;
    public static final double rotationalDeadband = Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond
        * 0.07;

  }

  public static class IntakeConstants {
    public static final int kWheelMotorId = 14;
    public static final int kPivotMotorId = 15;
    public static final int kIntakeSensorId = 8;

    public static final double kExtendSetpoint = -14.07;
    public static final double kRetractSetpoint = -3.7;
    public static final double kHeimlichSetpoint = -11.7;


    public static final double kPivotP = 100;
    public static final double kPivotI = 3;
    public static final double kPivotD = 4;
    public static final double kPivotG = 0;

    public static final double kStatorCurrentLimit = 80d;
    public static final double kHomeStatorCurrentLimit = 40d;
    public static final double kIntakeHomeDutyCycle = 0.1d;

    public static final boolean kTopIntakeMotorInverted = false;
    public static final boolean kBottomIntakeMotorInverted = false;
  }

  public static class EndeffectorConstants {

    public static final int kEndeffectorMotorId = 3;
    public static final int kEndeffectorSensorId = 0;

    public static final double kEndeffectorSpit = -0.75;

    public static final double kEndeffectorSlurp = 0.75;

  }

  public static class ArmConstants {
    public static final int kArmMotorId = 24;

    public static final double kArmP = 10000;
    public static final double kArmI = 5000;
    public static final double kArmD = 200;
    public static final double kArmS = 4;
    public static final double kArmV = 15;
    public static final double kArmA = 10;

    public static final double kArmG = 20;

    public static final double kArmCruiseVelocity = 1d;
    public static final GravityTypeValue kArmgravtype = GravityTypeValue.Arm_Cosine;
    public static final int kArmEncoderId = 30;
    public static final double kArmHeight = 5d;
    public static final double kArmWidth = 3.5;
    public static final double kArmRootX = 1d;
    public static final double kArmRootY = 2d;
    public static final double kArmRotationtoDegreeRatio = 2498d;

    public static final double kArmRestSetpoint = -.254;
    public static final double kArmAcceleration = 10d;
    public static final double kArmRotortoSensorRatio = (159d / 15d) * (36d / 12d);

    public static final double kArmHumanPlayer = 165;

  }

  public static class ElevatorConstants {
    public static final int kRightElevatorMotorId = 36;
    public static final int kLeftElevatorMotorId = 35;

    public static final double kElevatorP = 5000;
    public static final double kElevatorI = 20;
    public static final double kElevatorD = 220;
    public static final double kElevatorV = 0;
    public static final double kElevatorA = 0;
    public static final double kElevatorG = 15;
    public static final double kElevatorS = 0;

    public static final double kElevatorCruiseVelocity = 2;
    public static final double kElevatorAcceleration = 12.5;

    public static final int kLimitSwitchId = 9;

    public static final double kMaxExtension = 1.6;// cm
    public static final double kMinExtension = .3;// cm


    public static final double kElevatorMaxCurrent = 40.0d;
    public static final double kElevatorRegStatorCurrentLimit = 40.0d;
    public static final double kElevatorHomeStatorCurrentLimit = 40.0d;
    public static final double kElevatorHomeDutyCycle = 0.2;

    public static final int kRightElevatorEncoderId = 3;
    public static final int kLeftElevatorEncoderId = 3;

    public static final double kRotationsToMeters = (1/0.0313)/(.57/.536);//*Detroit Reference */;
    
    public static final double kElevatorWeight = 24d;
    public static final double kElevatorHeight = 1d;
    public static final double kElevatorWidth = 1d;
    public static final double kRootElevatorX = .3;
    public static final double kRootElevatorY = 0d;
    public static final double kRootElevator2X = .6;
    public static final double kRootElevator2Y = 0d;
    public static final double kLigaLength = .3d;
    public static final double kElevatorGearing = 14 / 72d;
    public static final double kDrumRadius = Units.inchesToMeters(1);
    public static final boolean kIsLeftInvert = true;

    public static final double kHomePosition = 0.83;
    public static final double kSafeArmElevatorPosition = kHomePosition;
    public static final double kElevatorPeakReverseDutyCycle = -0.7;
  }

  // WE ARE WELDEDkg
  public static class VisionConstants {
    public static final Vector<N3> kPrecisionInMyVision = VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(100));
    public static final String kLimelightScoreSide = "limelight-score";
    public static final String kLimelightOtherName = "limelight-right";
    public static final String kLimelightCoralName = "TheBEEPEE";
    public static final double kVisionAngularThreshold = 22.5;
    public static final double kTAThresholdFour = 3d;
    public static final double kTAThresholdThree = 3d;
    public static final double kTxThresholdCoral = 5;

  }

  public static class ScoringConstants {


    // Align Poses
    public static final Pose2d k17BlueLReefPose = new Pose2d(4, 3.7, new Rotation2d(0));
    public static final Pose2d k17BlueRReefPose = new Pose2d(4.366, 3.568, new Rotation2d(0));
    public static final Pose2d k18BlueRReefPose = new Pose2d(4, 4.2, new Rotation2d(180 - 180));
    public static final Pose2d k18BlueLReefPose = new Pose2d(4, 3.9, new Rotation2d(180 - 180));
    public static final Pose2d k19BlueLReefPose = new Pose2d(4.396, 4.522, new Rotation2d(121.5 - 180));
    public static final Pose2d k19BlueRReefPose = new Pose2d(4.124, 4.373, new Rotation2d(121.5 - 180));
    public static final Pose2d k20BlueRReefPose = new Pose2d(4.6, 4.516, new Rotation2d(60 - 180));
    public static final Pose2d k20BlueLReefPose = new Pose2d(4.86, 4.36, new Rotation2d(60 - 180));
    public static final Pose2d k21BlueLReefPose = new Pose2d(5, 4.15, new Rotation2d(0 - 180));
    public static final Pose2d k21BlueRReefPose = new Pose2d(5, 3.875, new Rotation2d(0 - 180));
    public static final Pose2d k22BlueLReefPose = new Pose2d(4.9, 3.65, new Rotation2d(-65 - 180));
    public static final Pose2d k22BlueRReefPose = new Pose2d(4.64, 3.54, new Rotation2d(-65 - 180));

    // PtP Poses
    // TODO acc find these if your going to use them
    public static final Pose2d k17BlueLReefPosePtP = new Pose2d(3.5, 2.66, new Rotation2d(155));
    public static final Pose2d k17BlueRReefPosePtP = new Pose2d(3.82, 2.48, new Rotation2d(155));
    public static final Pose2d k18BlueRReefPosePtP = new Pose2d(4, 4.2, new Rotation2d(180 - 180));
    public static final Pose2d k18BlueLReefPosePtP = new Pose2d(4, 3.9, new Rotation2d(180 - 180));
    public static final Pose2d k19BlueLReefPosePtP = new Pose2d(4.396, 4.522, new Rotation2d(121.5 - 180));
    public static final Pose2d k19BlueRReefPosePtP = new Pose2d(4.124, 4.373, new Rotation2d(121.5 - 180));
    public static final Pose2d k20BlueRReefPosePtP = new Pose2d(4.6, 4.516, new Rotation2d(60 - 180));
    public static final Pose2d k20BlueLReefPosePtP = new Pose2d(4.86, 4.36, new Rotation2d(60 - 180));
    public static final Pose2d k21BlueLReefPosePtP = new Pose2d(5, 4.15, new Rotation2d(0 - 180));
    public static final Pose2d k21BlueRReefPosePtP = new Pose2d(5, 3.875, new Rotation2d(0 - 180));
    public static final Pose2d k22BlueLReefPosePtP = new Pose2d(4.9, 3.65, new Rotation2d(-65 - 180));
    public static final Pose2d k22BlueRReefPosePtP = new Pose2d(4.64, 3.54, new Rotation2d(-65 - 180));

    // reset pose
    public static final Pose2d kResetPose = new Pose2d(2.84, 4, new Rotation2d(0));

    // Cage poses
    public static final Pose2d kCageTop = new Pose2d(8.8, 7.3, new Rotation2d(0d));
    public static final Pose2d kCageMiddle = new Pose2d(8.76, 6.150, new Rotation2d(0d));
    public static final Pose2d kCageBottom = new Pose2d(8.79, 5.06, new Rotation2d(0d));


    public static final double spitTimeout  = 2d;

    
    public static class L1 {
      public static final double kArmScoringPosition = .37;
      public static final double kElevatorStart = 0.5;
    public static final double kArmSafePosition = 0;
    public static final double kElevatorEnd = 0;
    }

    public static class L2 {
      //Homepos
      public static final double kElevatorStart =0.87;
      public static final double kArmScoringPosition = .376;
      public static final double kElevatorEnd = 0.541;
      public static final double kArmSafePosition = 0;

    }

    public static class L3 {
      public static final double kElevatorPos = .947;
      public static final double kArmScoringPosition = .379;

    }

    public static class L4 {
      public static final double kElevatorPos = 1.5;
      public static final double kArmScoringPosition = .354;

    }

    public static class Stow{
      public static final double kElevatorPos = 0.81;
      public static final double kArmStowPos = -0.25;
    }

    public static class Schloop{
      public static final double kElevatorPos = 0.645;
      public static final double kArmSchloPos = -0.25;

    }
    


    public static class Source {
      public static final double kElevatorPos = 1.32;
      public static final double kArmSourcePosition = -0.145;


    }
  }

  public static class SwerveConstants {
    // Both sets of gains need to be tuned to your individual robot.

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
      public static final PIDConstants translationPID = new PIDConstants(25, 0, 0);
      public static final PIDConstants rotationPID = new PIDConstants(2.5, 0, 0);

      public static final double kSlurpTimeout = 3d;

      public static class AutoPoses {
        public static final Pose2d kOpenPose = new Pose2d(7.557, 7.479, new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d kCenterPose = new Pose2d(7.557, 4.023, new Rotation2d(Units.degreesToRadians(180)));
        public static final Pose2d kProPose = new Pose2d(7.557, 0.685, new Rotation2d(Units.degreesToRadians(180)));

      }

      private final static MomentOfInertia kRobotMomentOfInertia = KilogramSquareMeters.of(0.01);

      private final static ModuleConfig kModuleConfig = new ModuleConfig(
          Constants.SwerveConstants.kWheelRadius,
          Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts,
          1.916,
          DCMotor.getKrakenX60Foc(1).withReduction(Constants.SwerveConstants.kDriveGearRatio),
          Amps.of(120),
          1);
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
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.73);

        // The maximum acceleration of the robot in meters per second squared.
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // The maximum angular speed of the robot in radians per second.
        public static final double kMaxAngularSpeedRadiansPerSecond = 13.5;

        // The maximum angular acceleration of the robot in radians per second squared.
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 3;

        public static final double kReefAdjustspeed = 0.25;
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
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.009765625);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(12.125);
    private static final Distance kFrontLeftYPos = Inches.of(12.125);

    // Front Right
    private static final int kFrontRightDriveMotorId = 22;
    private static final int kFrontRightSteerMotorId = 23;
    private static final int kFrontRightEncoderId = 21;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.324951171875);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(12.125);
    private static final Distance kFrontRightYPos = Inches.of(-12.125);

    // Back Left
    private static final int kBackLeftDriveMotorId = 32;
    private static final int kBackLeftSteerMotorId = 33;
    private static final int kBackLeftEncoderId = 31;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.2265625);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-12.125);
    private static final Distance kBackLeftYPos = Inches.of(12.125);

    // Back Right
    private static final int kBackRightDriveMotorId = 42;
    private static final int kBackRightSteerMotorId = 43;
    private static final int kBackRightEncoderId = 41;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.4990234375);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;

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

    public static class TunerConstants {

      /**
       * Creates a CommandSwerveDrivetrain instance.
       * This should only be called once in your robot program,.
       */


      /**
       * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected
       * device types.
       */
      public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct
         * the devices themselves. If they need the devices, they can access them
         * through
         * getters in the classes.
         *
         * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * @param modules             Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
          super(
              TalonFX::new, TalonFX::new, CANcoder::new,
              drivetrainConstants, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct
         * the devices themselves. If they need the devices, they can access them
         * through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
          super(
              TalonFX::new, TalonFX::new, CANcoder::new,
              drivetrainConstants, odometryUpdateFrequency, modules);
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not
         * construct
         * the devices themselves. If they need the devices, they can access them
         * through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve
         *                                  drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz
         *                                  on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry
         *                                  calculation
         *                                  in the form [x, y, theta]ᵀ, with units in
         *                                  meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision
         *                                  calculation
         *                                  in the form [x, y, theta]ᵀ, with units in
         *                                  meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
          super(
              TalonFX::new, TalonFX::new, CANcoder::new,
              drivetrainConstants, odometryUpdateFrequency,
              odometryStandardDeviation, visionStandardDeviation, modules);
        }
      }
    }

  }

}
