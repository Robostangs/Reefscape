
package frc.robot.commands.SwerveCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;



public class PathToPoint extends SequentialCommandGroup {
        private CommandSwerveDrivetrain drivetrain;
        private Pose2d targetPose;

        /**
         * 
        * This command drives the robot from tis current position to a target pose,
        * using set speed and acceleration limits.

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
                                                                Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                                                                .in(MetersPerSecond),
                                                                Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAccelerationMetersPerSecondSquared,
                                                                Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond,
                                                                Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularAccelerationRadiansPerSecondSquared),
                                                0.0));

        }
}
