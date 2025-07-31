package frc.robot.Stable;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.Factories.IntakeFactory;
import frc.robot.commands.Factories.ScoringFactory;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.SwerveCommands.AligntoReef;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class AutoManager {
    public final ArrayList<ScoringTargets> AutoScoringSides = new ArrayList<>();
    private final Constants.SwerveConstants.AutoConstants.AutoStartPosition start;
    private int pieces = 0;
    int pathcount = 0;
    private Command autoCommand = new PrintCommand("Default Auto Command");

    private PathPlannerAuto pathPlannerAuto = new PathPlannerAuto(autoCommand, new Pose2d());

    /**
     * Constructor for AutoManager.
     *
     * @param start The starting position for the autonomous routine.
     * @param sides A list of maps representing scoring sides and their boolean
     *              states.
     * @throws IllegalArgumentException if start or sides is null.
     */
    public AutoManager(Constants.SwerveConstants.AutoConstants.AutoStartPosition start,
            ArrayList<ScoringTargets> sides) {
        if (start == null) {
            throw new IllegalArgumentException("Start position cannot be null.");
        }
        if (sides == null) {
            throw new IllegalArgumentException("Sides list cannot be null.");
        }
        this.start = start;
        AutoScoringSides.clear();

        for (ScoringTargets side : sides) {
            if (side != null) {
                AutoScoringSides.add(side);
                pieces++;
            }
        }
        SmartDashboard.putNumber("What piece auto", pieces);
        setAutoCommand();

    }

    /**
     * Sets the autonomous command based on the starting position and scoring map.
     */
    public void setAutoCommand() {

        if (start == Constants.SwerveConstants.AutoConstants.AutoStartPosition.ShitinShit) {
            autoCommand = new PrintCommand("Pooping!!!!");
        } else if (start == Constants.SwerveConstants.AutoConstants.AutoStartPosition.Mobility) {
            autoCommand = CommandSwerveDrivetrain.getInstance().applyRequest(() -> new SwerveRequest.RobotCentric()
                    .withVelocityX(
                            Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond)
                                    * 0.3))
                    .withTimeout(1);
        } else {
            for (int c = 0; c < AutoScoringSides.size(); c++) {
                final int index = c;

                ScoringTargets currentSide = AutoScoringSides.get(index);

                autoCommand = autoCommand.andThen(AligntoReef.getDriveToReef(
                        () -> currentSide.isRight(),
                        currentSide.getAprilTag()).alongWith(
                                ScoringFactory.L4PositionAuto()))
                        .andThen(new Spit().withTimeout(0.4))
                        .andThen(ScoringFactory.SmartStow().withTimeout(0.2))
                        .andThen(
                        
                        (AutoBuilder.pathfindThenFollowPath(
                                start == Constants.SwerveConstants.AutoConstants.AutoStartPosition.Pro
                                        ? Constants.SwerveConstants.AutoConstants.AutoPaths.kProcessorCleanup
                                        : Constants.SwerveConstants.AutoConstants.AutoPaths.kOpenCleanup,
                                Constants.SwerveConstants.AutoConstants.AutoPaths.constraints)
                        
                                .raceWith(
                                        IntakeFactory.IntakeCoral())))
                        .andThen(new Retract().withTimeout(0.5));

                // autoCommand = autoCommand.andThen(AligntoReef.getDriveToReef(
                // () -> currentSide.isRight(),
                // currentSide.getAprilTag()).andThen(
                // ScoringFactory.L4Position().withTimeout(5)))
                // .andThen(new Spit().withTimeout(0.5))
                // .andThen(ScoringFactory.SmartStow().withTimeout(1))
                // .andThen(AutoBuilder.pathfindThenFollowPath(
                // start == Constants.SwerveConstants.AutoConstants.AutoStartPosition.Pro
                // ? Constants.SwerveConstants.AutoConstants.AutoPaths.kProcessorCleanup
                // : Constants.SwerveConstants.AutoConstants.AutoPaths.kOpenCleanup,
                // Constants.SwerveConstants.AutoConstants.AutoPaths.constraints)
                // .raceWith(
                // IntakeFactory.IntakeCoral()))
                // .onlyIf(() -> start !=
                // Constants.SwerveConstants.AutoConstants.AutoStartPosition.Center);

                pathcount++;

            }
        }
        SmartDashboard.putNumber("Path count", pathcount);
    }

    public static void putPieceChoosers(SendableChooser<Constants.SwerveConstants.AutoConstants.ReefSides> chooser) {
        for (Constants.SwerveConstants.AutoConstants.ReefSides side : Constants.SwerveConstants.AutoConstants.ReefSides
                .values()) {
            chooser.addOption(side.toString(), side);
        }
    }

    public static void putStartChooser(
            SendableChooser<Constants.SwerveConstants.AutoConstants.AutoStartPosition> chooser) {
        for (Constants.SwerveConstants.AutoConstants.AutoStartPosition position : Constants.SwerveConstants.AutoConstants.AutoStartPosition
                .values()) {
            chooser.addOption(position.toString(), position);
        }
    }

    public Command getAutoCommand() {
        return autoCommand;
    }

}