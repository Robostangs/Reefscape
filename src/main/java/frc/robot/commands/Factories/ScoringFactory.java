package frc.robot.commands.Factories;

import java.lang.Thread.State;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.MoveArm;
import frc.robot.commands.ElevatorCommands.Lift;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.subsystems.Elevator;

public class ScoringFactory {

    public enum ScoringPosition {
        L1, L2, L3, L4, Stow, Schloop
    }

    static Alert stowAlert = new Alert("Can't go to stow", Alert.AlertType.kError);

    static ScoringPosition ScoreState = ScoringPosition.Stow;

    public static Command L1Position() {
        return new Lift(Constants.ScoringConstants.L1.kElevatorStart)
                .andThen(new MoveArm(Constants.ScoringConstants.L1.kArmScoringPosition))
                .andThen(new Lift(Constants.ScoringConstants.L1.kElevatorEnd)).finallyDo(() -> {
                    ScoreState = ScoringPosition.L1;
                });

    }

    public static Command L2Position() {
        ScoreState = ScoringPosition.L2;
        return new Lift(Constants.ScoringConstants.L2.kElevatorStart).andThen(
                new MoveArm(Constants.ScoringConstants.L2.kArmScoringPosition))
                .andThen(new Lift(Constants.ScoringConstants.L2.kElevatorEnd)).finallyDo(() -> {
                    ScoreState = ScoringPosition.L2;
                });
    }

    public static Command L3Position() {
        ScoreState = ScoringPosition.L3;
        return new Lift(Constants.ScoringConstants.L3.kElevatorPos)
                .alongWith(new WaitUntilCommand(
                        () -> Elevator.getInstance()
                                .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                        .andThen(new MoveArm(Constants.ScoringConstants.L3.kArmScoringPosition)))
                .finallyDo(() -> {
                    ScoreState = ScoringPosition.L3;
                });
    }

    public static Command L4Position() {
        ScoreState = ScoringPosition.L4;
        return new Lift(Constants.ScoringConstants.L4.kElevatorPos)
                .alongWith(
                        new WaitUntilCommand(
                                () -> Elevator.getInstance()
                                        .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                                .andThen(
                                        new MoveArm(Constants.ScoringConstants.L4.kArmScoringPosition)))
                .finallyDo(
                        () -> {
                            ScoreState = ScoringPosition.L4;
                        });
    }

    public static Command L1Score() {
        return L1Position().andThen(new Spit()).withTimeout(Constants.ScoringConstants.spitTimeout);
    }

    public static Command L2Score() {
        return L2Position().andThen(new Spit()).withTimeout(Constants.ScoringConstants.spitTimeout);
    }

    public static Command L3Score() {
        return L3Position().andThen(new Spit()).withTimeout(Constants.ScoringConstants.spitTimeout);
    }

    public static Command L4Score() {
        return L4Position().andThen(new Spit()).withTimeout(Constants.ScoringConstants.spitTimeout);
    }

    public static Command Stow() {
        if (ScoreState == ScoringPosition.L2) {
            return new Lift(Constants.ScoringConstants.Stow.kElevatorPos)
                    .andThen(new MoveArm(Constants.ScoringConstants.Stow.kArmStowPos));
        } else if ((ScoreState == ScoringPosition.L3) || (ScoreState == ScoringPosition.L4)) {
            return new MoveArm(Constants.ArmConstants.kArmRestSetpoint).andThen(
                    new Lift(Constants.ScoringConstants.Stow.kElevatorPos));
        } else {
            stowAlert.set(true);
            return new PrintCommand("Can't stow");
        }

    }

    public static Command StowL2() {
        return new Lift(Constants.ScoringConstants.Stow.kElevatorPos)
                .andThen(new MoveArm(Constants.ScoringConstants.Stow.kArmStowPos));
    }

    public static Command SchloopCommand() {
        return new MoveArm(Constants.ArmConstants.kArmRestSetpoint).andThen(
                new Lift(Constants.ScoringConstants.Schloop.kElevatorPos));

    }

    public static Runnable returnStow() {
        return () -> {
            new MoveArm(Constants.ArmConstants.kArmRestSetpoint).andThen(
                    new Lift(Constants.ScoringConstants.Stow.kElevatorPos));
        };
    }

    public static Runnable returnStowL2() {
        return () -> {
            new Lift(Constants.ScoringConstants.Stow.kElevatorPos)
                    .andThen(new MoveArm(Constants.ScoringConstants.Stow.kArmStowPos));
        };
    }

    public static Runnable getCoral() {
        return () -> {
            new MoveArm(Constants.ArmConstants.kArmRestSetpoint).andThen(
                    new Lift(Constants.ScoringConstants.Source.kElevatorPos));
        };
    }
}