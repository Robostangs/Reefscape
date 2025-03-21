/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;

public class AligntoReef extends Command {
  private PIDController xController, yController;
  Supplier<Rotation2d> getTargetRotation;

  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveRequest.RobotCentricFacingAngle driveRequest;
  private CommandSwerveDrivetrain drivetrain;
  private double tagID = -1;
  Pose2d targetPose;
  AprilTagFields map;
  AprilTagFieldLayout theMap;
  double AprilTagID;
  Alert reefNoSeeID;

  public AligntoReef(boolean isRightScore) {
    xController = new PIDController(1, 0.0, 2); // Vertical movement
    yController = new PIDController(5, 0.0, 4); // Horitontal movement

    this.isRightScore = isRightScore;

    reefNoSeeID = new Alert("Lock in lil bro you can't see no april tag", AlertType.kError);

    this.drivetrain = CommandSwerveDrivetrain.getInstance();
    addRequirements(drivetrain);
  }

  public Pose2d getTargetPose() {
    Pose2d targetPose;
    try {
      targetPose = getReefPose();

    } catch (Exception e) {
      targetPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
    }

    if (isRightScore) {
      targetPose = targetPose.transformBy(
          new Transform2d(new Translation2d(Units.inchesToMeters(7 - 10), 0),
              Rotation2d.fromDegrees(0)));
    } else {
      targetPose = targetPose.transformBy(
          new Transform2d(new Translation2d(Units.inchesToMeters(-7 - 10), 0),
              Rotation2d.fromDegrees(0)));
    }
    return targetPose;

  }

  public Pose2d getReefPose() {
    try {
      if (!Robot.isSimulation()) {
        map = AprilTagFields.k2025ReefscapeWelded;
        theMap = AprilTagFieldLayout.loadField(map);

        AprilTagID = LimelightHelpers.getFiducialID(Constants.VisionConstants.kLimelightFour);
        if (AprilTagID == -1) {
          reefNoSeeID.set(true);
          throw new Exception("Invalid AprilTag ID");
        }
      } else {
        map = AprilTagFields.k2025ReefscapeWelded;
        theMap = AprilTagFieldLayout.loadField(map);

        AprilTagID = 17;
      }

      reefNoSeeID.set(false);
      return theMap.getTagPose((int) AprilTagID).get().toPose2d();
    } catch (Exception e) {
      reefNoSeeID.set(true);
      return CommandSwerveDrivetrain.getInstance().getState().Pose;
    }
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    targetPose = getTargetPose();

    getTargetRotation = () -> {

      return (targetPose.getRotation().plus(Rotation2d.fromDegrees(90)));
    };

    Robot.teleopField.getObject("Reef Align Pose")
        .setPose(targetPose);

    driveRequest = new SwerveRequest.RobotCentricFacingAngle();
    driveRequest.HeadingController = new PhoenixPIDController(24, 0, 1);

    drivetrain.postStatus("Aligning to Reef");

    xController.setSetpoint(getTargetPose().getX());
    xController.setTolerance(Constants.VisionConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(getTargetPose().getY());
    yController.setTolerance(Constants.VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID(Constants.VisionConstants.kLimelightFour);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV(Constants.VisionConstants.kLimelightFour)
        && LimelightHelpers.getFiducialID(Constants.VisionConstants.kLimelightFour) == tagID) {
      this.dontSeeTagTimer.reset();

      double xSpeed = xController.calculate(drivetrain.getState().Pose.getX());
      double ySpeed = -yController.calculate(drivetrain.getState().Pose.getX());

      SmartDashboard.putNumber("Align/ Target X", xController.getSetpoint());
      SmartDashboard.putNumber("Align/Position X", drivetrain.getState().Pose.getX());

      SmartDashboard.putNumber("Align/ Target Y", yController.getSetpoint());
      SmartDashboard.putNumber("Align/Position Y", drivetrain.getState().Pose.getY());

      driveRequest.TargetDirection = getTargetRotation.get();

      driveRequest.withVelocityX(xSpeed);
      driveRequest.withVelocityY(ySpeed);

      drivetrain.setControl(driveRequest);

      if (!yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.postStatus("Aligned to Reef");
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long
    // as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(1) ||
        stopTimer.hasElapsed(0.3);
  }
}
