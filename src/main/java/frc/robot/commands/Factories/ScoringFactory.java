package frc.robot.commands.Factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.MoveArm;
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
                .andThen(new MoveArm(Constants.ScoringConstants.kArmScoringangle)
                        .andThen(new Spit()));
    }

    public static Command L4Score() {
        return new Lift(Constants.ScoringConstants.L4.kElevatorPos).andThen(
                new MoveArm(Constants.ScoringConstants.kArmScoringangle));
    }

    public static Command L1Position() {
        return new Lift(Constants.ScoringConstants.L1.kElevatorPos)
                .andThen(new Extend())
                .andThen(new MoveArm(Constants.ScoringConstants.kArmScoringangle));
    }

    public static Command L2Position() {
        return new Lift(Constants.ScoringConstants.L2.kElevatorPos).andThen(
                new MoveArm(Constants.ScoringConstants.kArmScoringangle));
    }

    public static Command L3Position() {
        return new Lift(Constants.ScoringConstants.L3.kElevatorPos)
                .andThen(new MoveArm(Constants.ScoringConstants.kArmScoringangle));
    }

    public static Command L4Position() {
        return new Lift(Constants.ScoringConstants.L4.kElevatorPos).andThen(
                new MoveArm(Constants.ScoringConstants.kArmScoringangle));
    }

    public static Command returnHome(){
        return new MoveArm(Constants.ArmConstants.kArmRestsetpoint).andThen(
            new Lift(0d));
        
    }

    public static Runnable returnHomeRun() {
        return () -> {
            new MoveArm(Constants.ArmConstants.kArmRestsetpoint).andThen(
                    new Lift(0d));
        };
    }
}