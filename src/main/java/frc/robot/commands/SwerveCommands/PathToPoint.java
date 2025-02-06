
package frc.robot.commands.SwerveCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathToPoint extends SequentialCommandGroup {
        private CommandSwerveDrivetrain drivetrain;
        private Pose2d targetPose;

        /**
         * Uses {@code AutoBuilder} to drive to {@code targetPose} 
         * 
         * @param targetPose the position that the robot should move to
         */
        public PathToPoint(Pose2d targetPose) {
                drivetrain = CommandSwerveDrivetrain.getInstance();

                this.targetPose = targetPose;

                this.setName("PathToPoint");
                this.addRequirements(drivetrain);

                this.addCommands(
                                AutoBuilder.pathfindToPoseFlipped(
                                                this.targetPose,
                                                new PathConstraints(
                                                                Constants.TunerConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.magnitude(),
                                                                Constants.TunerConstants.AutoConstants.AutoSpeeds.kMaxAccelerationMetersPerSecondSquared,
                                                                Constants.TunerConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond,
                                                                Constants.TunerConstants.AutoConstants.AutoSpeeds.kMaxAngularAccelerationRadiansPerSecondSquared),
                                                0.0));
        }
}
