package frc.robot.commands.Factories;

import java.util.Set;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ArmCommands.MoveArm;
import frc.robot.commands.ElevatorCommands.Lift;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class ScoringFactory {

    public enum ScoringPosition {
        L1, L2, L3, L4, Stow, Schloop, Algeeeee, Source
    }

    static Alert stowAlert = new Alert("Can't go to stow", Alert.AlertType.kError);
    static Alert algeAlert = new Alert("Can't knock out alge", Alert.AlertType.kError);

    public static ScoringPosition ScoreState = ScoringPosition.Stow;

    public static Command L2Position() {
        return new Lift(Constants.ScoringConstants.L2.kElevatorStart).andThen(
                new MoveArm(Constants.ScoringConstants.L2.kArmScoringPosition))
                .andThen(new Lift(Constants.ScoringConstants.L2.kElevatorEnd)
                        .finallyDo(() -> {
                            ScoreState = ScoringPosition.L2;
                        }));
    }

    public static Command L3Position() {
        return new Lift(Constants.ScoringConstants.L3.kElevatorPos)
                .alongWith(new WaitUntilCommand(
                        () -> Elevator.getInstance()
                                .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                        .onlyIf(() -> !Robot.isSimulation())
                        .andThen(new MoveArm(Constants.ScoringConstants.L3.kArmScoringPosition))
                        .finallyDo(() -> {
                            ScoreState = ScoringPosition.L3;
                        }));
    }

    public static Command L4Position() {
        return new Lift(Constants.ScoringConstants.L4.kElevatorPos)
                .alongWith(
                        new WaitUntilCommand(
                                () -> Elevator.getInstance()
                                        .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                                .onlyIf(() -> !Robot.isSimulation())
                                .andThen(
                                        new MoveArm(Constants.ScoringConstants.L4.kArmScoringPosition))
                                .finallyDo(() -> {
                                    ScoreState = ScoringPosition.L4;
                                }));
    }

    public static Command ByeByeByeAlge(ScoringPosition position) {
        switch (position) {
            case L2:

                return new Lift(Constants.ScoringConstants.L2.kELevatorAlgepos)
                        .andThen(new MoveArm(Constants.ScoringConstants.L2.kArmAlgePos)
                        .finallyDo(() -> ScoreState = ScoringPosition.Algeeeee));
                        

            case L3:
                return new Lift(Constants.ScoringConstants.L3.kELevatorAlgepos)
                        .andThen(new MoveArm(Constants.ScoringConstants.L3.kArmAlgePos)
                                .finallyDo(() -> ScoreState = ScoringPosition.Algeeeee));
            default:
                algeAlert.set(true);
                return new PrintCommand("Can't go to alge");
        }
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

    public static Command StowL2() {
        return new Lift(Constants.ScoringConstants.Stow.kElevatorPos)
                .andThen(new MoveArm(Constants.ScoringConstants.Stow.kArmStowPos));
    }

    public static Command Stow() {
        return new DeferredCommand(() -> {
            if (ScoreState.equals(ScoringPosition.L2)) {
                return StowL2().finallyDo(() -> {
                    ScoreState = ScoringPosition.Stow;
                });
            } else {
                return new MoveArm(Constants.ArmConstants.kArmRestSetpoint).andThen(
                        new Lift(Constants.ScoringConstants.Stow.kElevatorPos)
                                .finallyDo(() -> {
                                    ScoreState = ScoringPosition.Stow;
                                }));
            }

        }, Set.of(Arm.getInstance(), Elevator.getInstance()));

    }

    public static Command SourceIntake() {
        return new Lift(Constants.ScoringConstants.Source.kElevatorPos)
                .andThen(new MoveArm(Constants.ScoringConstants.Source.kArmSourcePosition)
                        .finallyDo(() -> ScoreState = ScoringPosition.Source));
    }

    public static Command SchloopCommand() {
        return new MoveArm(Constants.ScoringConstants.Stow.kArmStowPos).andThen(
                new Lift(Constants.ScoringConstants.Schloop.kElevatorPos).finallyDo(() -> {
                    ScoreState = ScoringPosition.Schloop;
                }));

    }

    public static Runnable returnStow() {
        return () -> {
            Stow();
        };
    }

    public static Runnable returnStowL2() {
        return () -> {
            StowL2();
        };
    }

    public static Runnable getSchloop() {
        return () -> {
            SchloopCommand();
        };
    }
}