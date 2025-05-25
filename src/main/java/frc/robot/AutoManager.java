package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.AutoConstants.ReefSides;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.Factories.ScoringFactory;
import frc.robot.commands.SwerveCommands.AligntoReef;

public class AutoManager {
    ArrayList<Constants.SwerveConstants.AutoConstants.ReefSides> sides = new ArrayList<>();
    public ArrayList<Map<Constants.SwerveConstants.AutoConstants.ReefSides, Boolean>> AutoScoringMap;
    Constants.SwerveConstants.AutoConstants.AutoStartPosition start;
    int pieces;
    private Command autoCommand;

    public AutoManager(Constants.SwerveConstants.AutoConstants.AutoStartPosition start,
            List<Map<Constants.SwerveConstants.AutoConstants.ReefSides, Boolean>> sides) {
        for (Map<ReefSides, Boolean> side : sides) {
            this.AutoScoringMap.add(side);
            pieces++;
        }
        this.start = start;
        setAutoCommand();
    }

    public void setAutoCommand() {
        boolean isRight = false;
        for (int c = 0; c < AutoScoringMap.size(); c++) {
            final int index = c;

            if (AutoScoringMap.get(index).containsValue(Boolean.TRUE)) {
                isRight = true;
            } else {
                isRight = false;
            }

            final boolean finalIsRight = isRight;

            autoCommand.andThen(AligntoReef.getDriveToReef(
                    () -> finalIsRight,
                    getReefID(AutoScoringMap.get(index).keySet().iterator().next()))).alongWith(
                            (ScoringFactory.L4PositionAuto()))
                    .andThen(new Spit().withTimeout(0.4))
                    .andThen(
                            ScoringFactory.SmartStow())
                    .andThen(AutoBuilder.pathfindThenFollowPath(
                            start == Constants.SwerveConstants.AutoConstants.AutoStartPosition.Pro
                                    ? Constants.SwerveConstants.AutoConstants.AutoPaths.kOpenCleanup
                                    : Constants.SwerveConstants.AutoConstants.AutoPaths.kOpenCleanup,
                            Constants.SwerveConstants.AutoConstants.AutoPaths.constraints));
        }
    }

    public int getReefID(Constants.SwerveConstants.AutoConstants.ReefSides side) {
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
                    return -1;
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
                    return -1;
            }
        }

    }

    public Command getAutoCommand() {
        return autoCommand;
    }

}
