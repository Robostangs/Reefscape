package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.*;

/*
 * This command alligns the robot to a coral on the field using a Limelight camera.
 * It uses the Limelight to calculate the degree fo the turn and continues to turn.
 * while moving, it adjusts the robot's angle to face the coral.
 * This command never finsihes on its own and must be stopped manually (battery kill).
 */

public class AligntoCoral extends Command {

    CommandSwerveDrivetrain drivetrain;

    SwerveRequest.FieldCentricFacingAngle driveRequest;

    Supplier<Double> translateX, translateY;
    Supplier<Rotation2d> getTargetRotation;
    String llName;

    //

    public AligntoCoral() {

        drivetrain = CommandSwerveDrivetrain.getInstance();

        this.addRequirements(drivetrain);

        this.translateX = () -> 0d;
        this.translateY = () -> 0d;
        llName = Constants.VisionConstants.kLimelightCoralName;

        this.setName("Align to Coral");

        getTargetRotation = () -> {
            double degreeOffset = LimelightHelpers.getTX(llName);
            return new Rotation2d(degreeOffset);
        };

    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(llName, 2);
        driveRequest = new SwerveRequest.FieldCentricFacingAngle();
        driveRequest.HeadingController = new PhoenixPIDController(6, 0, 1);

        // this is for tuning and now we can tune the PID controller
        SmartDashboard.putData("Align to Coral PID", driveRequest.HeadingController);
        drivetrain.postStatus("Aligning to Coral");

        driveRequest.Deadband = Constants.OperatorConstants.kDriverDeadband;
        driveRequest.RotationalDeadband = Constants.OperatorConstants.rotationalDeadband;
    }

    @Override
    public void execute() {

        driveRequest.TargetDirection = getTargetRotation.get();

        if (Math.abs(LimelightHelpers.getTX(llName)) < 6) {
            driveRequest
                    .withVelocityX(translateX.get()
                            * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond))
                    .withVelocityY(
                            Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond);
        } else {
            driveRequest
                    .withVelocityX(translateX.get()
                            * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond))
                    .withVelocityY(translateY.get()
                            * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond);
        }
        drivetrain.setControl(driveRequest);

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.postStatus("Aligned to Coral");
    }

    @Override
    public boolean isFinished() {
        if (Robot.isSimulation()) {
            return false;
        }

        else {
            return false;
        }

    }
}
