package frc.robot.commands.SwerveCommands;

import java.util.stream.Collectors;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;
import com.therekrab.autopilot.Autopilot.APResult;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
public class AutopilotAlign extends Command {
    private final APTarget m_target;
    private static final AprilTagFieldLayout theMap;
    private final boolean isRight;
    static {

      AprilTagFields map = AprilTagFields.k2025ReefscapeWelded;

      theMap = AprilTagFieldLayout.loadField(map);
    }

    private CommandSwerveDrivetrain m_drivetrain = CommandSwerveDrivetrain.getInstance();
    private final SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withHeadingPID(Constants.AutoConstants.AutopilotConstants.kP, Constants.AutoConstants.AutopilotConstants.kI, Constants.AutoConstants.AutopilotConstants.kD); /* tune this for your robot! */
  
  
    public AutopilotAlign(boolean isRight) {
      this.isRight = isRight;
      m_target = new APTarget(getTargetPose(isRight));
      

      m_drivetrain = CommandSwerveDrivetrain.getInstance();
      addRequirements(m_drivetrain);
    }
  
    @Override
    public void initialize() {
      /* no-op */
    }
  
    @Override
    public void execute() {
      ChassisSpeeds robotRelativeSpeeds = m_drivetrain.getState().Speeds;
      Pose2d pose = m_drivetrain.getPose();
  
      APResult out = Constants.AutoConstants.AutopilotConstants.kAutopilot.calculate(pose, robotRelativeSpeeds, m_target);
      System.out.println("Target Angle: " + out.targetAngle());
      System.out.println("Current Angle: " + pose.getRotation());
      m_drivetrain.setControl(m_request
          .withVelocityX(out.vx())
          .withVelocityY(out.vy())
          .withTargetDirection(out.targetAngle()));
    }
  
    @Override
    public boolean isFinished() {
      return Constants.AutoConstants.AutopilotConstants.kAutopilot.atTarget(m_drivetrain.getPose(), m_target);
    }
  
    @Override
    public void end(boolean interrupted) {
      m_drivetrain.stopDrivetrain();
    }

    public static Pose2d getReefPose() {


      Pose2d currentPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
      
      if (Robot.isRed()) {
        return currentPose.nearest(
            theMap.getTags().subList(5, 12).stream().map((ting) -> ting.pose.toPose2d())
                .collect(Collectors.toList()));
      } else {
        return currentPose.nearest(
            theMap.getTags().subList(16, 22).stream().map((ting) -> ting.pose.toPose2d())
                .collect(Collectors.toList()));

      }

  }
  public static Pose2d getTargetPose(boolean isRight) {
    Pose2d targetPose = getReefPose();

    // take the reef pose and move it to the right or left
    if (isRight) {
      targetPose = targetPose.transformBy(
          new Transform2d(
              new Translation2d(Constants.VisionConstants.ReefAlign.kTagRelativeXOffset,
                  Constants.VisionConstants.ReefAlign.kTagRelativeYOffsetRight),
              Rotation2d.fromDegrees(90)));
    } else {
      targetPose = targetPose.transformBy(
          new Transform2d(
              new Translation2d(Constants.VisionConstants.ReefAlign.kTagRelativeXOffset,
                  Constants.VisionConstants.ReefAlign.kTagRelativeYOffsetLeft),
              Rotation2d.fromDegrees(90)));
    }

    return targetPose;

  }
}