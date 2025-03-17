package frc.robot.commands.SwerveCommands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.MetersPerSecond;
import frc.robot.commands.SwerveCommands.AligntoReef;

public class AligntoReef extends Command {

    CommandSwerveDrivetrain drivetrain;

    SwerveRequest.RobotCentricFacingAngle driveRequest;


    Supplier<Double> translateX, translateY;
    Supplier<Rotation2d> getTargetRotation;
    int AprilTagID;
    boolean Right;
    Pose2d targetPose;
    AprilTagFields map;
    AprilTagFieldLayout theMap;

    public AligntoReef(boolean Right) {

        drivetrain = CommandSwerveDrivetrain.getInstance();

        this.Right = Right;

        this.addRequirements(drivetrain);

        getTargetRotation = () -> {

            return getTargetPose().getRotation();
        };

        this.setName("Align to Reef");

    }

    public Pose2d getTargetPose() {
        Pose2d targetPose = getReefPose();


  
            if (Right) {
                targetPose = targetPose.transformBy(
                        new Transform2d(new Translation2d(Units.inchesToMeters(7 - 10), 0),
                                Rotation2d.fromDegrees(270)));
            } else {
                targetPose = targetPose.transformBy(
                        new Transform2d(new Translation2d(Units.inchesToMeters(-7 - 10), 0),
                                Rotation2d.fromDegrees(270)));
        }
        return targetPose;

    }

    public Pose2d getReefPose() {

        if (!Robot.isSimulation()) {
            map = AprilTagFields.k2025ReefscapeWelded;

            theMap = AprilTagFieldLayout.loadField(map);

            AprilTagID = LimelightHelpers.getRawFiducials(Constants.VisionConstants.kLimelightScoreSide)[0].id;

        } else {
            map = AprilTagFields.k2025ReefscapeWelded;

            theMap = AprilTagFieldLayout.loadField(map);

            AprilTagID = 17;

        }
        return theMap.getTagPose(AprilTagID).get().toPose2d();

    }

    // target pose is rotate and translate in tag space
    @Override
    public void initialize() {

        driveRequest = new SwerveRequest.RobotCentricFacingAngle();
        driveRequest.HeadingController = new PhoenixPIDController(6, 0, 1);

        drivetrain.postStatus("Aligning to Reef");

        Robot.teleopField.getObject("Reef Align Pose")
                .setPose(new Pose2d(getTargetPose().getX(), getTargetPose().getY(), getTargetPose().getRotation()));

    }

    @Override
    public void execute() {

        targetPose = getTargetPose();

        driveRequest.TargetDirection = getTargetRotation.get();

        if(getTargetPose().getX() - CommandSwerveDrivetrain.getInstance().getState().Pose.getX() > 0){
            driveRequest.withVelocityX(-Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond)*0.2);
        }
        else{
            driveRequest.withVelocityX(Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond)*0.2);
        }


        if(getTargetPose().getY() - CommandSwerveDrivetrain.getInstance().getState().Pose.getY() > 0){
            driveRequest.withVelocityY(-Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond)*0.2);
        }
        else{
            driveRequest.withVelocityY(Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond)*0.2);
        }

        drivetrain.setControl(driveRequest);



        SmartDashboard.putNumber("Drivetrain/April Tag ID", AprilTagID);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.postStatus("Aligned");
    }

    @Override
    public boolean isFinished() {

        return (Math.abs(drivetrain.getPose().getY() - targetPose.getY()) < 0.1)
                && (Math.abs(drivetrain.getPose().getX() - targetPose.getX()) < 0.1);

    }
}
