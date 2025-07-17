/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.MetersPerSecond;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class AligntoReef {

  private static final AprilTagFieldLayout theMap;
  static {

    AprilTagFields map = AprilTagFields.k2025ReefscapeWelded;

    theMap = AprilTagFieldLayout.loadField(map);
  }

  /**
   * Generates a path to the reef
   * <p>
   * NOTE: The rotations of that poses in this method are NOT
   * the rotation of the robot but the rotation that the robot
   * should be going to be heading towards(the control points in path planner)
   * 
   * @param targetPose the target pose to generate a path to
   * 
   */
  public static PathPlannerPath generatePath(Pose2d targetPose, boolean Auto) {

    Pose2d currentPose = CommandSwerveDrivetrain.getInstance().getState().Pose;



    // start pose should be your X and Y but the rotation should be where your
    // heading to
    Pose2d startPose = new Pose2d(currentPose.getX(), currentPose.getY(),
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle().minus(Rotation2d.k180deg));

    // ending pose should be the reef X and Y but the rotation should be where your
    // heading to
    Pose2d endPose = new Pose2d(targetPose.getX(), targetPose.getY(),
        targetPose.getRotation().plus(Rotation2d.kCCW_90deg));

    List<Waypoint> waypoints;

    // if the robot rotation isn't that off from the target rotation then it should
    // be a simple path
    if (Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 45) {
      waypoints = PathPlannerPath.waypointsFromPoses(
          startPose, endPose);
    }
    // if the robot rotation is off by a lot then we should add a point before so we
    // have time to rotate
    else {
      waypoints = PathPlannerPath.waypointsFromPoses(startPose,
          endPose.transformBy(new Transform2d(Units.inchesToMeters(18), 0, Rotation2d.kZero)), endPose);
    }

    PathConstraints constraints = new PathConstraints(
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond) * 0.4,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond * 0.4,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAccelerationMetersPerSecondSquared * 0.4,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularAccelerationRadiansPerSecondSquared * 0.4);

    PathConstraints endconstraints = new PathConstraints(
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond) * 0.1,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond * 0.1,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAccelerationMetersPerSecondSquared * 0.1,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularAccelerationRadiansPerSecondSquared * 0.1);

    List<RotationTarget> rotationTargets = new ArrayList<RotationTarget>();
    // this is what the rotation of the actual robot should be
    rotationTargets.add(new RotationTarget(0.8, endPose.getRotation().plus(Rotation2d.fromDegrees(270))));

    List<ConstraintsZone> zones = new ArrayList<ConstraintsZone>();
    zones.add(new ConstraintsZone(0.5, 1, endconstraints));
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        rotationTargets,
        Collections.emptyList(),
        zones,
        Collections.emptyList(),
        constraints,
        null,
        new GoalEndState(0.0, targetPose.getRotation()), false);

    path.preventFlipping = true;

    return path;
  }

  public static PathPlannerPath generatePathAuto(Pose2d targetPose) {

    Pose2d currentPose = CommandSwerveDrivetrain.getInstance().getState().Pose;

    // start pose should be your X and Y but the rotation should be where your
    // heading to
    SmartDashboard.putNumber("Autos/Current Pose Calculated", currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle()
.minus(Rotation2d.k180deg).getDegrees());

    SmartDashboard.putNumber("Autos/End Pose Calculated", targetPose.getRotation().plus(Rotation2d.kCCW_90deg).getDegrees());

    SmartDashboard.putNumber("Autos/Current Pose ", currentPose.getTranslation().getAngle().getDegrees());

    SmartDashboard.putNumber("Autos/End Pose ", targetPose.getRotation().getDegrees());
    
    Pose2d startPose = new Pose2d(currentPose.getX(), currentPose.getY(),
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle().minus(Rotation2d.k180deg));

    // ending pose should be the reef X and Y but the rotation should be where your
    // heading to
    Pose2d endPose = new Pose2d(targetPose.getX(), targetPose.getY(),  Rotation2d.fromDegrees(
        targetPose.getRotation().plus(Rotation2d.kCCW_90deg).getDegrees()));

    List<Waypoint> waypoints;

    // if the robot rotation isn't that off from the target rotation then it should
    // be a simple path
    if (Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 45) {
      waypoints = PathPlannerPath.waypointsFromPoses(
          startPose, endPose);

      
    }
    // if the robot rotation is off by a lot then we should add a point before so we
    // have time to rotate
    else {
      waypoints = PathPlannerPath.waypointsFromPoses(startPose,
          endPose.transformBy(new Transform2d(Units.inchesToMeters(-18), 0, Rotation2d.kZero)), endPose);
    }

    PathConstraints constraints = new PathConstraints(
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond),
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAccelerationMetersPerSecondSquared,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularAccelerationRadiansPerSecondSquared);

    PathConstraints endconstraints = new PathConstraints(
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond) * 0.1,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond * 0.1,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAccelerationMetersPerSecondSquared * 0.1,
        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularAccelerationRadiansPerSecondSquared * 0.1);

    List<RotationTarget> rotationTargets = new ArrayList<RotationTarget>();
    // this is what the rotation of the actual robot should be
    rotationTargets.add(new RotationTarget(0.7, endPose.getRotation().plus(Rotation2d.fromDegrees(270))));

    List<ConstraintsZone> zones = new ArrayList<ConstraintsZone>();
    zones.add(new ConstraintsZone(0.7, 1, endconstraints));
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        rotationTargets,
        Collections.emptyList(),
        zones,
        Collections.emptyList(),
        constraints,
        null,
        new GoalEndState(0.0, targetPose.getRotation()), false);

    path.preventFlipping = true;

    return path;
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

  public static Pose2d getTargetPose(boolean isRight, int tagID) {
    Pose2d targetPose = theMap.getTagPose(tagID).get().toPose2d();

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

  /**
   * Aligns the robot to the reef using path planner
   * 
   * @param isRight align to the right or left branch of the reef
   */
  public static Command getAlignToReef(BooleanSupplier isRight) {

    return new DeferredCommand(() -> {
      Robot.teleopField.getObject("Reef Align Pose").setPose(getTargetPose((isRight.getAsBoolean())));

      return AutoBuilder.followPath(generatePath(getTargetPose(isRight.getAsBoolean()), false));
    },
        Set.of(CommandSwerveDrivetrain.getInstance()));

  }

  /**
   * Aligns the robot to the reef using path planner based on what april tag you
   * provide
   * 
   * @param isRight align to the right or left branch of the reef
   */
  public static Command getDriveToReef(BooleanSupplier isRight, int tagID) {

    return new DeferredCommand(() -> {

      PathPlannerPath path = generatePathAuto(getTargetPose(isRight.getAsBoolean(), tagID));

      Robot.teleopField.getObject("Auto Target").setPose(getTargetPose((isRight.getAsBoolean())));
      Robot.teleopField.getObject("Current Path").setPoses(getTargetPose(isRight.getAsBoolean(), tagID));

      Robot.teleopField.getObject("THE PATH").setPoses(path.getPathPoses());

      return AutoBuilder.followPath(path);
    },
        Set.of(CommandSwerveDrivetrain.getInstance()));

  }

}
