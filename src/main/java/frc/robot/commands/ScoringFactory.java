package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ElevatorCommands.Lift;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.IntakeCommands.Extend;

public class ScoringFactory {

    public static Command L1Score() {
        return new Lift(Constants.ScoringConstants.L1.kElevatorPos)
        .andThen(new Extend())
        .andThen(new MoveArm(Constants.ScoringConstants.kArmScoringangle)
                        .andThen(new Spit()));
    }

    public static Command L2Score() {
        return new Lift(Constants.ScoringConstants.L2.kElevatorPos).andThen(
                new MoveArm(Constants.ScoringConstants.kArmScoringangle).andThen(
                        new Spit()));
    }

    public static Command L3Score() {
        return new Lift(Constants.ScoringConstants.L3.kElevatorPos)
        .andThen( new MoveArm(Constants.ScoringConstants.kArmScoringangle)
        .andThen(new Spit()));
    }

    public static Command L4Score() {
        return new Lift(Constants.ScoringConstants.L4.kElevatorPos).andThen(
                new MoveArm(Constants.ScoringConstants.kArmScoringangle).andThen(
                        new Spit()));
    }

    public static Runnable returnHome() {
        return () -> {
            new MoveArm(0d).andThen(
                    new Lift(0d));
        };
    }
}