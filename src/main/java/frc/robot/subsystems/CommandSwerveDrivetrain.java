package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends Constants.SwerveConstants.TunerConstants.TunerSwerveDrivetrain
        implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    // private SimSwerveDrivetrain m_simDrivetrain;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
    // new SwerveRequest.SysIdSwerveSteerGains();
    // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
    // new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.ApplyRobotSpeeds AutoDrive = new SwerveRequest.ApplyRobotSpeeds();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    // private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
    // new SysIdRoutine.Config(
    // null, // Use default ramp rate (1 V/s)
    // Volts.of(7), // Use dynamic voltage of 7 V
    // null, // Use default timeout (10 s)
    // // Log state with SignalLogger class
    // state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
    // new SysIdRoutine.Mechanism(
    // volts -> setControl(m_steerCharacterization.withVolts(volts)),
    // null,
    // this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    // private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
    // new SysIdRoutine.Config(
    // /* This is in radians per second², but SysId only supports "volts per second"
    // */
    // Volts.of(Math.PI / 6).per(Second),
    // /* This is in radians per second, but SysId only supports "volts" */
    // Volts.of(Math.PI),
    // null, // Use default timeout (10 s)
    // // Log state with SignalLogger class
    // state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
    // new SysIdRoutine.Mechanism(
    // output -> {
    // /* output is actually radians per second, but SysId only supports "volts" */
    // setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
    // /* also log the requested output for SysId */
    // SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
    // },
    // null,
    // this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

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
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        this.setVisionMeasurementStdDevs(Constants.VisionConstants.kErrorInMyVision);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configurePathPlanner();

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
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

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
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.me
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // for (SwerveModule<TalonFX, TalonFX, CANcoder> swerveModule : getModules()) {
        // if (Robot.verifyMotor(swerveModule.getDriveMotor())) {
        // swerveModule.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
        // }
        // if (Robot.verifyMotor(swerveModule.getSteerMotor())) {
        // swerveModule.getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
        // }

        // }

        /*
         * Only seed ll4 with external yaw angle if not rotating fast, otherwise use
         * internal imu with mode 2
         */
        if (this.getPigeon2().getAngularVelocityZWorld()
                .getValueAsDouble() > Constants.VisionConstants.kLL4SeedMaxWz) {
            /*
             * 0 - Use external IMU yaw submitted via SetRobotOrientation() for MT2
             * localization. The internal IMU is ignored entirely.
             * 1 - Use external IMU yaw submitted via SetRobotOrientation(), and configure
             * the LL4 internal IMU’s fused yaw to match the submitted yaw value.
             * 2 - Use internal IMU for MT2 localization. External imu data is ignored
             * entirely
             */
            LimelightHelpers.SetIMUMode(Constants.VisionConstants.kLimelightFour, 2);
        } else {
            LimelightHelpers.SetIMUMode(Constants.VisionConstants.kLimelightFour, 1);

            LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.kLimelightFour,
                    this.getState().Pose.getRotation().getDegrees(),
                    0d,
                    0d,
                    0d,
                    0d,
                    0d);

        }

        /** Seed the three unconditionally bc it doesn't got that dawg(imu) in him */
        LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.kLimelightThree,
                this.getState().Pose.getRotation().getDegrees(),
                0d,
                0d,
                0d,
                0d,
                0d);

        if (DriverStation.isDisabled()) {
            NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.kLimelightFour)
                    .getEntry("throttle-set").setNumber(200);
        } else {
            NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.kLimelightFour)
                    .getEntry("throttle-set").setNumber(0);
        }
        if (!Robot.isSimulation()) {



            LimelightHelpers.PoseEstimate fourPoseEsti, threePoseEsti;
            if (DriverStation.isDisabled()) {

                fourPoseEsti = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.VisionConstants.kLimelightFour);
                threePoseEsti = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.VisionConstants.kLimelightThree);
            } else if (this.getPigeon2().getAngularVelocityZWorld()
                    .getValueAsDouble() < Constants.VisionConstants.kVisionAngularThreshold) {

                fourPoseEsti = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.kLimelightFour);
                threePoseEsti = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.kLimelightThree);
            } else {
                fourPoseEsti = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.kLimelightFour);
                threePoseEsti = null;

            }

            if (isPosegoo(threePoseEsti)) {
                this.addVisionMeasurement(threePoseEsti.pose, threePoseEsti.timestampSeconds);
                Robot.teleopField.getObject("Limelight Three Pose").setPose(threePoseEsti.pose);
            }

            if (isPosegoo(fourPoseEsti)) {
                this.addVisionMeasurement(fourPoseEsti.pose, fourPoseEsti.timestampSeconds);
                Robot.teleopField.getObject("Limelight Four Pose").setPose(fourPoseEsti.pose);
            }

        }

        SmartDashboard.putBoolean("Are we using vision", RobotContainer.useVision);

    }

    public static Runnable toggleVision = () -> {
        RobotContainer.useVision = false;

    };


    private boolean isPosegoo(LimelightHelpers.PoseEstimate yoPoseEsti) {
        if (yoPoseEsti == null) {
            return false;
        }
        if (yoPoseEsti.tagCount < 1) {
            return false;
        }

        SmartDashboard.putNumber("Error Distance", yoPoseEsti.pose.getTranslation().minus(this.getState().Pose.getTranslation()).getNorm());

        return RobotContainer.useVision;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Drivetrain/status", status);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }
    public void stopDrivetrain(){
        this.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0,0,0),
        this.getPose().getRotation())));
    }
   
    public void configurePathPlanner() {
        AutoBuilder.configure(
                () -> this.getState().Pose,
                this::resetPose,
                () -> this.getState().Speeds,
                (speeds, feedforwards) -> this.setControl(AutoDrive.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())),
                new PPHolonomicDriveController(Constants.AutoConstants.translationPID,
                        Constants.AutoConstants.rotationPID),
                Constants.AutoConstants.robotConfig,
                () -> Robot.isRed(),
                this);

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            if (poses.size() > 0)
                Robot.teleopField
                        .getObject("Trajectory").setPoses(poses);
        });

    }

    private static CommandSwerveDrivetrain mDrivetrain;

    public static CommandSwerveDrivetrain getInstance() {
        if (mDrivetrain == null) {
            mDrivetrain = new CommandSwerveDrivetrain(Constants.SwerveConstants.DrivetrainConstants,
                    Constants.SwerveConstants.FrontLeft,
                    Constants.SwerveConstants.FrontRight, Constants.SwerveConstants.BackLeft,
                    Constants.SwerveConstants.BackRight);
        }
        return mDrivetrain;
    }
    


  public Command align(APTarget target) {
    return this.run(
            () -> {
              SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
                    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withHeadingPID(Constants.AutoConstants.AutopilotConstants.kP, Constants.AutoConstants.AutopilotConstants.kI, Constants.AutoConstants.AutopilotConstants.kD);
              SmartDashboard.putNumberArray("TARGET_POSE", new double[]{ target.getReference().getMeasureX().baseUnitMagnitude(), target.getReference().getMeasureY().baseUnitMagnitude(), target.getReference().getRotation().getDegrees() });

              ChassisSpeeds robotRelativeSpeeds = this.getState().Speeds;
              Pose2d pose = this.getPose();

              APResult output = Constants.AutoConstants.AutopilotConstants.kAutopilot.calculate(pose, robotRelativeSpeeds, target);

              /* these speeds are field relative */
              double veloX = output.vx().in(MetersPerSecond);
              double veloY = output.vy().in(MetersPerSecond);
              double headingReference = output.targetAngle().getRadians();
              double diff = headingReference-pose.getRotation().getRadians();
              if (diff > Math.PI) {
                diff -= 360;
              } else if (diff < -Math.PI) {
                diff += 360;
              }

              double appliedRot = Math.abs(diff) > Units.degreesToRadians(2) ? (diff * Constants.AutoConstants.AutopilotConstants.kP_ROT) : 0;
              SmartDashboard.putNumber("currentRot", pose.getRotation().getDegrees());
              Robot.teleopField.getObject("Autopilot Pose").setPose(target.getReference());
              SmartDashboard.putNumber("headingTarget", headingReference);
              SmartDashboard.putNumber("sub", diff);
              SmartDashboard.putNumber("appliedRot", appliedRot);

              this.setControl(m_request
              .withVelocityX(output.vx())
              .withVelocityY(output.vy())
              .withTargetDirection(output.targetAngle()));        })
        .until(() -> 
        Constants.AutoConstants.AutopilotConstants.kAutopilot.atTarget(this.getPose(), target)
        )
        .finallyDo(() -> this.stopDrivetrain());
  }
    
}
