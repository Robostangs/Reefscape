/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public class AligntoReef {

  public static PathPlannerPath generatePath(Pose2d targetPose) {

    Pose2d currentPose = CommandSwerveDrivetrain.getInstance().getState().Pose;
    Pose2d startPose = new Pose2d(currentPose.getX(), currentPose.getY(),
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle().minus(Rotation2d.k180deg));
    Pose2d endPose = new Pose2d(targetPose.getX(), targetPose.getY(),
        targetPose.getRotation().plus(Rotation2d.kCCW_90deg));
    List<Waypoint> waypoints;

    if (Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 45) {
      waypoints = PathPlannerPath.waypointsFromPoses(
          startPose, endPose);
    } else {
      waypoints = PathPlannerPath.waypointsFromPoses(startPose,
          endPose.transformBy(new Transform2d(18, 0, Rotation2d.kZero)), endPose);
    }

    PathConstraints constraints = new PathConstraints(
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond),
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAccelerationMetersPerSecondSquared,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularAccelerationRadiansPerSecondSquared);

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        new GoalEndState(0.0, targetPose.getRotation()));

    path.getRotationTargets().add(new RotationTarget(0.8, endPose.getRotation()));

    path.preventFlipping = true;

    return path;
  }

  public static Pose2d getTargetPose(boolean isRight) {
    Pose2d targetPose = getReefPose();

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

  public static Pose2d getReefPose() {

    AprilTagFields map = AprilTagFields.k2025ReefscapeWelded;

    AprilTagFieldLayout theMap = AprilTagFieldLayout.loadField(map);

    Pose2d currentPose = CommandSwerveDrivetrain.getInstance().getState().Pose;

    if (Robot.isRed()) {
      return currentPose.nearest(
          theMap.getTags().subList(5, 12).stream().map((ting) -> ting.pose.toPose2d()).collect(Collectors.toList()));
    } else {
      return currentPose.nearest(
          theMap.getTags().subList(16, 22).stream().map((ting) -> ting.pose.toPose2d()).collect(Collectors.toList()));

    }
  }

  public static Command getAlignToReef(BooleanSupplier isRight) {

    return new DeferredCommand(() -> {
      Robot.teleopField.getObject("Reef Align Pose").setPose(getTargetPose((isRight.getAsBoolean())));

      return AutoBuilder.followPath(generatePath(getTargetPose(isRight.getAsBoolean())));
    },
        Set.of(CommandSwerveDrivetrain.getInstance()));

  }

}
