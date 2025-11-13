/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class AligntoReef {

  /*
   * This command helps allign to the reef using the april tags on the field.
   * It makes a path based on the current pose of the robot and the target pose of
   * the reef.
   * The reef position is slightly changed to go to the left or right side, based
   * on what you choose.
   */

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
  public static PathPlannerPath generatePath(Pose2d targetPose) {

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
        Constants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond) * 0.4,
        Constants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond * 0.4,
        Constants.AutoConstants.AutoSpeeds.kMaxAccelerationMetersPerSecondSquared * 0.4,
        Constants.AutoConstants.AutoSpeeds.kMaxAngularAccelerationRadiansPerSecondSquared * 0.4);

    PathConstraints endconstraints = new PathConstraints(
        Constants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond) * 0.1,
        Constants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond * 0.1,
        Constants.AutoConstants.AutoSpeeds.kMaxAccelerationMetersPerSecondSquared * 0.1,
        Constants.AutoConstants.AutoSpeeds.kMaxAngularAccelerationRadiansPerSecondSquared * 0.1);

    List<RotationTarget> rotationTargets = new ArrayList<RotationTarget>();
    // this is what the rotation of the actual robot should be
    rotationTargets.add(new RotationTarget(0.8, endPose.getRotation().plus(Rotation2d.fromDegrees(270))));

    List<ConstraintsZone> zones = new ArrayList<ConstraintsZone>();
    zones.add(new ConstraintsZone(0.6, 1, endconstraints));
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

  public static Command align(APTarget target ) {

    CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    return drivetrain.run(
            () -> {
              SwerveRequest.FieldCentricFacingAngle m_request = new SwerveRequest.FieldCentricFacingAngle()
                    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withHeadingPID(Constants.AutoConstants.AutopilotConstants.kP, Constants.AutoConstants.AutopilotConstants.kI, Constants.AutoConstants.AutopilotConstants.kD);
              SmartDashboard.putNumberArray("TARGET_POSE", new double[]{ target.getReference().getMeasureX().baseUnitMagnitude(), target.getReference().getMeasureY().baseUnitMagnitude(), target.getReference().getRotation().getDegrees() });

              ChassisSpeeds robotRelativeSpeeds = drivetrain.getState().Speeds;
              Pose2d pose = drivetrain.getPose();

              APResult output = Constants.AutoConstants.AutopilotConstants.kAutopilot.calculate(pose, robotRelativeSpeeds, target);

              /* these speeds are field relative */
              double veloX = output.vx().in(MetersPerSecond);
              double veloY = output.vy().in(MetersPerSecond);
              double headingReference = output.targetAngle().getRadians();
              double diff = headingReference-pose.getRotation().getRadians();
              if (diff > Math.PI) {
                diff -= 360;
              } else if (diff < -Math.PI) {
                diff += 360;
              }

              double appliedRot = Math.abs(diff) > Units.degreesToRadians(2) ? (diff * Constants.AutoConstants.AutopilotConstants.kP_ROT) : 0;
              SmartDashboard.putNumber("currentRot", pose.getRotation().getDegrees());
              Robot.teleopField.getObject("Autopilot Pose").setPose(target.getReference());
              SmartDashboard.putNumber("headingTarget", headingReference);
              SmartDashboard.putNumber("sub", diff);
              SmartDashboard.putNumber("appliedRot", appliedRot);

              drivetrain.setControl(m_request
              .withVelocityX(output.vx())
              .withVelocityY(output.vy())
              .withTargetDirection(output.targetAngle()));        })
        .until(() -> 
        Constants.AutoConstants.AutopilotConstants.kAutopilot.atTarget(drivetrain.getPose(), target)
        )
        .finallyDo(() -> drivetrain.stopDrivetrain());
  }


  /**
   * Aligns the robot to the reef using path planner
   * 
   * @param isRight align to the right or left branch of the reef
   */
  public static Command getAlignToReef(BooleanSupplier isRight) {

    return new DeferredCommand(() -> {
      Robot.teleopField.getObject("Reef Align Pose").setPose(getTargetPose((isRight.getAsBoolean())));

      return AutoBuilder.followPath(generatePath(getTargetPose(isRight.getAsBoolean())));
    },
        Set.of(CommandSwerveDrivetrain.getInstance()));

  }

  public static Command autopilotAlign(BooleanSupplier isRight) {

    return new DeferredCommand(() -> {
      return align(new APTarget(getTargetPose(isRight.getAsBoolean())));
    },
      Set.of(CommandSwerveDrivetrain.getInstance()));

  }



}