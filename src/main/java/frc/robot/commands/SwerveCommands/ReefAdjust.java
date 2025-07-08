package frc.robot.commands.SwerveCommands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class ReefAdjust extends Command {

/*
 * This command always adjusts the robot's position and orientation to align wih reef and coral by
 * adjusting the robot's angle until it is perfectly aimed. And it does all of that by using vision input.
 */
    CommandSwerveDrivetrain drivetrain;

    SwerveRequest.FieldCentricFacingAngle driveRequest;

    Supplier<Rotation2d> getTargetRotation;

    public ReefAdjust() {

        drivetrain = CommandSwerveDrivetrain.getInstance();

        this.addRequirements(drivetrain);

        this.setName("Align to Coral");

        getTargetRotation = () -> {
            double DegreeOffset = LimelightHelpers.getTX(Constants.VisionConstants.kLimelightFour);
            return new Rotation2d(DegreeOffset + 90);
        };
    }

    @Override
    public void initialize() {
        driveRequest = new SwerveRequest.FieldCentricFacingAngle();
        driveRequest.HeadingController = new PhoenixPIDController(6, 0, 1);

        // this is for tuning and now we can tune the PID controller
        SmartDashboard.putData("Align to Coral PID", driveRequest.HeadingController);
        drivetrain.postStatus("Aligning to Reef");

        driveRequest.Deadband = Constants.OperatorConstants.kDriverDeadband;
        driveRequest.RotationalDeadband = Constants.OperatorConstants.rotationalDeadband;

    }

    @Override
    public void execute() {
        if (Math.abs(LimelightHelpers
                .getTX(Constants.VisionConstants.kLimelightFour)) >= Constants.VisionConstants.kTxThresholdCoral) {
            if (LimelightHelpers.getTX(Constants.VisionConstants.kLimelightFour) > 0) {
                driveRequest.VelocityY = 2.3;
            } else if (LimelightHelpers.getTX(Constants.VisionConstants.kLimelightFour) < 0) {
                driveRequest.VelocityY = -2.3;
            }
        } else {
            driveRequest.VelocityY = 0;
        }

     
        drivetrain.setControl(driveRequest);

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.postStatus("Aligned");
        driveRequest.VelocityY = 0;

    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return false;
        }

        else {
            return !(Math.abs(LimelightHelpers.getTX(
                    Constants.VisionConstants.kLimelightFour)) >= Constants.VisionConstants.kTxThresholdCoral);
        }

    }
}
