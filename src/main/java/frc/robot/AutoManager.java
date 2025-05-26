package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.SwerveConstants.AutoConstants.ReefSides;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.Factories.ScoringFactory;
import frc.robot.commands.SwerveCommands.AligntoReef;

public class AutoManager {
    private final ArrayList<Constants.SwerveConstants.AutoConstants.ReefSides> sides = new ArrayList<>();
    public final ArrayList<Map<Constants.SwerveConstants.AutoConstants.ReefSides, Boolean>> AutoScoringMap = new ArrayList<>();
    private final Constants.SwerveConstants.AutoConstants.AutoStartPosition start;
    private int pieces = 0;
    private Command autoCommand = new PrintCommand("Default Auto Command");

    /**
     * Constructor for AutoManager.
     *
     * @param start The starting position for the autonomous routine.
     * @param sides A list of maps representing scoring sides and their boolean
     *              states.
     * @throws IllegalArgumentException if start or sides is null.
     */
    public AutoManager(Constants.SwerveConstants.AutoConstants.AutoStartPosition start,
            ArrayList<Map<Constants.SwerveConstants.AutoConstants.ReefSides, Boolean>> sides) {
        if (start == null) {
            throw new IllegalArgumentException("Start position cannot be null.");
        }
        if (sides == null) {
            throw new IllegalArgumentException("Sides list cannot be null.");
        }

        for (Map<ReefSides, Boolean> side : sides) {
            if (side != null) {
                this.AutoScoringMap.add(side);
                pieces++;
            }
        }
        this.start = start;
        setAutoCommand();
    }

    /**
     * Sets the autonomous command based on the starting position and scoring map.
     */
    public void setAutoCommand() {
        boolean isRight = false;

        if (start == Constants.SwerveConstants.AutoConstants.AutoStartPosition.shitinshit) {
            autoCommand = new PrintCommand("Pooping!!!!");
        } else {
            for (int c = 0; c < AutoScoringMap.size(); c++) {
                final int index = c;

                Map<ReefSides, Boolean> currentMap = AutoScoringMap.get(index);
                if (currentMap == null || currentMap.isEmpty()) {
                    continue; // Skip null or empty maps
                }

                if (currentMap.containsValue(Boolean.TRUE)) {
                    isRight = true;
                } else {
                    isRight = false;
                }

                final boolean finalIsRight = isRight;

                autoCommand = autoCommand.andThen(AligntoReef.getDriveToReef(
                        () -> finalIsRight,
                        getReefID(currentMap.keySet().iterator().next())).andThen(
                                ScoringFactory.L4PositionAuto().withTimeout(1)))
                        .andThen(new Spit().withTimeout(0.4))
                        .andThen(ScoringFactory.SmartStow().withTimeout(1))
                        .andThen(AutoBuilder.pathfindThenFollowPath(
                                start == Constants.SwerveConstants.AutoConstants.AutoStartPosition.Pro
                                        ? Constants.SwerveConstants.AutoConstants.AutoPaths.kProcessorCleanup
                                        : Constants.SwerveConstants.AutoConstants.AutoPaths.kOpenCleanup,
                                Constants.SwerveConstants.AutoConstants.AutoPaths.constraints)
                                .onlyIf(() -> start != Constants.SwerveConstants.AutoConstants.AutoStartPosition.Center));
                ;

            }
        }
    }

    public Command getAutoCommand() {
        return autoCommand;
    }

    /**
     * Gets the reef ID based on the reef side.
     *
     * @param side The reef side.
     * @return The reef ID.
     * @throws IllegalArgumentException if side is null.
     */
    public int getReefID(Constants.SwerveConstants.AutoConstants.ReefSides side) {
        if (side == null) {
            throw new IllegalArgumentException("Reef side cannot be null.");
        }

        if (Robot.isRed()) {
            switch (side) {
                case Center1:
                    return 10;
                case Center2:
                    return 7;
                case Pro1:
                    return 11;
                case Pro2:
                    return 6;
                case Open1:
                    return 9;
                case Open2:
                    return 8;
                default:
                    return -1; // Handle unexpected cases
            }
        } else {
            switch (side) {
                case Center1:
                    return 18;
                case Center2:
                    return 21;
                case Pro1:
                    return 17;
                case Pro2:
                    return 22;
                case Open1:
                    return 19;
                case Open2:
                    return 20;
                default:
                    return -1; // Handle unexpected cases
            }
        }
    }
}