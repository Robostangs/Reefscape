package frc.robot.commands.SwerveCommands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

public class Allign extends Command {
    CommandSwerveDrivetrain drivetrain;


    SwerveRequest.FieldCentricFacingAngle driveRequest;
    Constants.AlignConstants.AlignType alignType;
    Supplier<Double> translateX, translateY;
    Supplier<Rotation2d> getTargetRotation;
    Constants.AlignConstants.AlignType cageID;
    String llName;

    public Allign(Constants.AlignConstants.AlignType alignType) {
        drivetrain = CommandSwerveDrivetrain.getInstance();


        this.addRequirements(drivetrain);
        if (alignType == Constants.AlignConstants.AlignType.AlignToCoral) {
            AlligntoCoral();
        }
    };
    
    public Allign(Constants.AlignConstants.AlignType alignType, Supplier<Double> translateX, Supplier<Double> translateY, Constants.AlignConstants.AlignType cageID) {
        drivetrain = CommandSwerveDrivetrain.getInstance();

        this.addRequirements(drivetrain);
        if (alignType == Constants.AlignConstants.AlignType.AlignToCoral) {
            AlligntoCoral();
        }
        else if (alignType == Constants.AlignConstants.AlignType.AlignToCageTop) {
            AlligntoCage(translateX, translateY, cageID);
        }
    };

    public void AlligntoCoral() {
        this.translateX = () -> 0d;
        this.translateY = () -> 0d;
        llName = Constants.VisionConstants.kLimelightCoralName;

        this.setName("Align to Coral");

        getTargetRotation = () -> {
            double degreeOffset = LimelightHelpers.getTX(llName);
            return new Rotation2d(degreeOffset);
        };
    }

    public void AlligntoCage(Supplier<Double> translateX, Supplier<Double> translateY, Constants.AlignConstants.AlignType cageID) {
        this.translateX = translateX;
        this.translateY = translateY;
        this.cageID = cageID;

        this.setName("Align to Cage");
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
