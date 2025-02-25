package frc.robot.commands.SwerveCommands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakePivot;

import static edu.wpi.first.units.Units.*;

import frc.robot.commands.SwerveCommands.AligntoReef;

public class AligntoReef extends Command {

    CommandSwerveDrivetrain drivetrain;

    SwerveRequest.FieldCentricFacingAngle driveRequest;

    Supplier<Double> translateX, translateY;
    Supplier<Rotation2d> getTargetRotation;
    int AprilTagID;
    boolean Right;
    Pose3d reefPose;
    AprilTagFields map;
    AprilTagFieldLayout theMap;

    public AligntoReef(boolean Right) {
        this(() -> 0d, () -> 0d, Right);
    }

    public AligntoReef(Supplier<Double> translateX, Supplier<Double> translateY, boolean Right) {

        drivetrain = CommandSwerveDrivetrain.getInstance();

        this.addRequirements(drivetrain);

        this.translateX = translateX;
        this.translateY = translateY;
        this.Right = Right;

        this.setName("Align to Reef");

        map = AprilTagFields.k2025ReefscapeWelded;

        theMap = AprilTagFieldLayout.loadField(map);

        AprilTagID = LimelightHelpers.getRawFiducials(Constants.VisionConstants.kLimelightScoreSide)[0].id;

        reefPose = theMap.getTagPose(AprilTagID).get();

        getTargetRotation = () -> {
            double deltaX = drivetrain.getPose().getX() - reefPose.getX();
            double deltaY = drivetrain.getPose().getY() - reefPose.getY();

            return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX) + Units.degreesToRadians(90));

        };
    }

    @Override
    public void initialize() {
        driveRequest = new SwerveRequest.FieldCentricFacingAngle();
        driveRequest.HeadingController = new PhoenixPIDController(6, 0, 1);

        // this is for tuning and now we can tune the PID controller
        SmartDashboard.putData("Align to Reef PID", driveRequest.HeadingController);
        drivetrain.postStatus("Aligning to Reef");

        driveRequest.Deadband = Constants.OperatorConstants.kDriverDeadband;
        driveRequest.RotationalDeadband = Constants.OperatorConstants.rotationalDeadband;
    }

    @Override
    public void execute() {

        driveRequest.TargetDirection = getTargetRotation.get();

        driveRequest
                .withVelocityX(translateX.get()
                        * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond))
                .withVelocityY(translateY.get()
                        * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond);
        drivetrain.setControl(driveRequest);
        Robot.teleopField.getObject("Reef Align Pose")
                .setPose(new Pose2d(reefPose.getX(), reefPose.getY(), reefPose.getRotation().toRotation2d()));
        SmartDashboard.putNumber("Drivetrain/April Tag ID", AprilTagID);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.postStatus("Aligned");
    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return false;
        }

        else {
            return !IntakePivot.getInstance().getIntakeSensor();
        }

    }
}
