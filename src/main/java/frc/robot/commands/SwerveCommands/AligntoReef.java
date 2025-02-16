package frc.robot.commands.SwerveCommands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import frc.robot.commands.SwerveCommands.AligntoReef;

public class AligntoReef extends Command {

    CommandSwerveDrivetrain drivetrain;

    SwerveRequest.FieldCentricFacingAngle driveRequest;

    Supplier<Double> translateX, translateY;
    Supplier<Rotation2d> getTargetRotation;
    int AprilTagID;
    boolean Right;
    Pose2d reefPose = new Pose2d(0, 0, new Rotation2d(0));

    public AligntoReef(Supplier<Double> translateX, Supplier<Double> translateY, int AprilTagID, boolean Right) {

        drivetrain = CommandSwerveDrivetrain.getInstance();

        this.addRequirements(drivetrain);

        this.translateX = translateX;
        this.translateY = translateY;
        this.AprilTagID = AprilTagID;
        this.Right = Right;

        this.setName("Align to Speaker");

        if (Right) {
            if (AprilTagID == 17 || AprilTagID == 11) {
                reefPose = Constants.VisionConstants.k17BlueRReefPose;
            } else if (AprilTagID == 18 || AprilTagID == 10) {
                reefPose = Constants.VisionConstants.k18BlueRReefPose;
            } else if (AprilTagID == 19 || AprilTagID == 9) {
                reefPose = Constants.VisionConstants.k19BlueRReefPose;
            } else if (AprilTagID == 20 || AprilTagID == 8) {
                reefPose = Constants.VisionConstants.k20BlueRReefPose;
            } else if (AprilTagID == 21 || AprilTagID == 7) {
                reefPose = Constants.VisionConstants.k21BlueRReefPose;
            } else if (AprilTagID == 22 || AprilTagID == 6) {
                reefPose = Constants.VisionConstants.k22BlueRReefPose;
            }
        } else {
            if (AprilTagID == 17 || AprilTagID == 11) {
                reefPose = Constants.VisionConstants.k17BlueLReefPose;
            } else if (AprilTagID == 18 || AprilTagID == 10) {
                reefPose = Constants.VisionConstants.k18BlueLReefPose;
            } else if (AprilTagID == 19 || AprilTagID == 9) {
                reefPose = Constants.VisionConstants.k19BlueLReefPose;
            } else if (AprilTagID == 20 || AprilTagID == 8) {
                reefPose = Constants.VisionConstants.k20BlueLReefPose;
            } else if (AprilTagID == 21 || AprilTagID == 7) {
                reefPose = Constants.VisionConstants.k21BlueLReefPose;
            } else if (AprilTagID == 22 || AprilTagID == 6) {
                reefPose = Constants.VisionConstants.k22BlueLReefPose;
            }
            if (Robot.isRed()) {
                FlippingUtil.flipFieldPose(reefPose);
            }
        }

        getTargetRotation = () -> {
            return Rotation2d
                    .fromRadians(Math.atan2(
                            drivetrain.getPose().getY() - reefPose.getY(),
                            drivetrain.getPose().getX() - reefPose.getX()));
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
        System.out.println("Aligning to Reef");

        driveRequest.TargetDirection = getTargetRotation.get();

        driveRequest
                .withVelocityX(translateX.get()
                        * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond))
                .withVelocityY(translateY.get()
                        * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond);
        drivetrain.setControl(driveRequest);
        Robot.teleopField.getObject("Reef Align Pose").setPose(reefPose);
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
            return !Intake.getInstance().getIntakeSensor();
        }

    }
}
