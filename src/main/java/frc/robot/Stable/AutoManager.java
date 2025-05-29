package frc.robot.Stable;

import java.util.ArrayList;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.Factories.ScoringFactory;
import frc.robot.commands.SwerveCommands.AligntoReef;

public class AutoManager {
    public final ArrayList<ScoringTargets> AutoScoringSides = new ArrayList<>();
    private final Constants.SwerveConstants.AutoConstants.AutoStartPosition start;
    private int pieces = 0;
    int pathcount = 0;
    private Command autoCommand= new PrintCommand("Default Auto Command");

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
        AutoScoringSides.clear();
        if (start == null) {
            throw new IllegalArgumentException("Start position cannot be null.");
        }
        if (sides == null) {
            throw new IllegalArgumentException("Sides list cannot be null.");
        }
        this.start = start; 


            for (ScoringTargets side : sides) {
                if (side != null) {
                    this.AutoScoringSides.add(side);
                    pieces++;
                }
            }
            SmartDashboard.putNumber("What piece auto", pieces);
            setAutoCommand();

        
    }

    public void clearAutoCommand() {
        autoCommand = new PrintCommand("Default Auto Command");
    }

    /**
     * Sets the autonomous command based on the starting position and scoring map.
     */
    public void setAutoCommand() {

        if (start == Constants.SwerveConstants.AutoConstants.AutoStartPosition.shitinshit) {
            autoCommand = new PrintCommand("Pooping!!!!");
        } 
        else {
            for (int c = 0; c < AutoScoringSides.size(); c++) {
                final int index = c;

                ScoringTargets currentSide = AutoScoringSides.get(index);
           


                autoCommand = autoCommand.andThen(AligntoReef.getDriveToReef(
                        () -> currentSide.isRight(),
                        currentSide.getAprilTag()).andThen(
                                ScoringFactory.L4Position().withTimeout(1)))
                        .andThen(new Spit().withTimeout(0.4))
                        .andThen(ScoringFactory.SmartStow().withTimeout(1))
                        .andThen(AutoBuilder.pathfindThenFollowPath(
                                start == Constants.SwerveConstants.AutoConstants.AutoStartPosition.Pro
                                        ? Constants.SwerveConstants.AutoConstants.AutoPaths.kProcessorCleanup
                                        : Constants.SwerveConstants.AutoConstants.AutoPaths.kOpenCleanup,
                                Constants.SwerveConstants.AutoConstants.AutoPaths.constraints)
                                .onlyIf(() -> start != Constants.SwerveConstants.AutoConstants.AutoStartPosition.Center));

                pathcount++;

            }
        }
        SmartDashboard.putNumber("Path count", pathcount);
    }

    public Command getAutoCommand() {
        return autoCommand;
    }

}