package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.*;

public class AligntoCage extends Command {

    CommandSwerveDrivetrain drivetrain;

    SwerveRequest.FieldCentricFacingAngle driveRequest;
    SwerveRequest.RobotCentric driveRequet;


    Supplier<Double> translateX, translateY;
    Supplier<Rotation2d> getTargetRotation;
    int cageID;
    Pose2d cagePose;
    Alert alignAlert;

    public AligntoCage(Supplier<Double> translateX, Supplier<Double> translateY, int cageID) {

        drivetrain = CommandSwerveDrivetrain.getInstance();

        this.addRequirements(drivetrain);

        this.translateX = translateX;
        this.translateY = translateY;
        this.cageID = cageID;
        alignAlert = new Alert("Invalid Cage ID", AlertType.kError);

        this.setName("Align to Coral");

        if (cageID == 1) {
            cagePose = Constants.ScoringConstants.kCageMiddle;
        } else if (cageID == 2) {
            cagePose = Constants.ScoringConstants.kCageBottom;

        } else if (cageID == 3) {
            cagePose = Constants.ScoringConstants.kCageMiddle;

        } else {
            cagePose = new Pose2d(0, 0, new Rotation2d());
            alignAlert.set(true);
        }

        if(Robot.isRed()){
            FlippingUtil.flipFieldPose(cagePose);
        }
        getTargetRotation = () -> {
            double deltaX = drivetrain.getPose().getX() - cagePose.getX();
            double deltaY = drivetrain.getPose().getY() - cagePose.getY();

            return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX) );

        };

    }

    @Override
    public void initialize() {
        driveRequest = new SwerveRequest.FieldCentricFacingAngle();
        driveRequest.HeadingController = new PhoenixPIDController(6, 0, 1);


        // this is for tuning and now we can tune the PID controller
        SmartDashboard.putData("Align to Cage PID", driveRequest.HeadingController);
        drivetrain.postStatus("Aligning to Cage");

        driveRequest.Deadband = Constants.OperatorConstants.kDriverDeadband;
        driveRequest.RotationalDeadband = Constants.OperatorConstants.rotationalDeadband;
    }

    @Override
    public void execute() {

        driveRequest.TargetDirection = getTargetRotation.get();

        driveRequest
                .withVelocityX(translateX.get()
                        * Constants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond))
                .withVelocityY(translateY.get()
                        * Constants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond);

        drivetrain.setControl(driveRequest);

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.postStatus("Aligned to Cage");
    }

    @Override
    public boolean isFinished() {
            return false;
        

    }
}
