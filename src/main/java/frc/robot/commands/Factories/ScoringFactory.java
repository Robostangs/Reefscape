package frc.robot.commands.Factories;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ArmCommands.SetArmPosition;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class ScoringFactory {

    public enum ScoringPosition {
        L1, L2, L3, L4, Stow, Schloop, Algaeeeee, Source
    }

    static Alert stowAlert = new Alert("Can't go to stow", Alert.AlertType.kError);
    static Alert algaeAlert = new Alert("Can't knock out algae", Alert.AlertType.kError);

    public static ScoringPosition ScoreState = ScoringPosition.Stow;

    /**
     * Returns a command that makes the elevator go to the limit switch, then moves
     * the arm to the scoring
     * position, and then back down to the L2 setpoint.
     * 
     * @return A command to move the elevator and arm to the L2 scoring position.
     */
    public static Command L2Position() {
        return new SetElevatorPosition(Constants.ScoringConstants.L2.kElevatorStart).andThen(
                new SetArmPosition(Constants.ScoringConstants.L2.kArmScoringPosition))
                .andThen(new SetElevatorPosition(Constants.ScoringConstants.L2.kElevatorEnd)
                        .finallyDo(() -> {
                            ScoreState = ScoringPosition.L2;
                        }));
    }

    /**
     * Returns a command that makes the elevator go to the L3 setpoint while moving
     * the arm to the scoring position,
     * but only if the elevator is high enough so the arm won't hit anything.
     * 
     * @return A command to move the elevator and arm to the L3 scoring position.
     */
    public static Command L3Position() {
        return new SetElevatorPosition(Constants.ScoringConstants.L3.kElevatorPos)
                .alongWith(new WaitUntilCommand(
                        () -> Elevator.getInstance()
                                .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                        .onlyIf(() -> !Robot.isSimulation())
                        .andThen(new SetArmPosition(Constants.ScoringConstants.L3.kArmScoringPosition))
                        .finallyDo(() -> {
                            ScoreState = ScoringPosition.L3;
                        }));
    }

    /**
     * Returns a command that makes the elevator go to the L4 setpoint while moving
     * the arm to the scoring position.
     *
     * @return A command to move the elevator and arm to the L4 scoring position.
     */
    public static Command L4Position() {
        return new SetElevatorPosition(Constants.ScoringConstants.L4.kElevatorPos)
                .alongWith(
                        new WaitUntilCommand(
                                () -> Elevator.getInstance()
                                        .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                                .onlyIf(() -> !Robot.isSimulation())
                                .andThen(
                                        new SetArmPosition(Constants.ScoringConstants.L4.kArmScoringPosition))
                                .finallyDo(() -> {
                                    ScoreState = ScoringPosition.L4;
                                }));
    }

    /**
     * Returns a command that makes the elevator go to the L3 setpoint then move the
     * arm up ot knock out algae
     * 
     * @return A command to knock out algae
     */
    public static Command ByeByeByeAlgae() {

        return new SetElevatorPosition(Constants.ScoringConstants.L3.kELevatorAlgaepos)
                .andThen(new SetArmPosition(Constants.ScoringConstants.L3.kArmAlgaePos)
                        .finallyDo(() -> ScoreState = ScoringPosition.Algaeeeee));

    }

    public static Command L2Score(BooleanSupplier manipBumper) {
        return L2Position().andThen(new Spit()).onlyWhile(manipBumper);
    }

    public static Command L3Score(BooleanSupplier manipBumper) {
        return L3Position().andThen(new Spit()).onlyWhile(manipBumper);
    }

    public static Command L4Score(BooleanSupplier manipBumper) {
        return L4Position().andThen(new Spit()).onlyWhile(manipBumper);
    }

    public static Command StowL2() {
        return new SetElevatorPosition(Constants.ScoringConstants.Stow.kElevatorPos)
                .andThen(new SetArmPosition(Constants.ScoringConstants.Stow.kArmStowPos));
    }

    
    public static Command Stow() {
        return new DeferredCommand(() -> {
            if (ScoreState.equals(ScoringPosition.L2)) {
                return StowL2().finallyDo(() -> {
                    ScoreState = ScoringPosition.Stow;
                });
            } else {
                return new SetArmPosition(Constants.ArmConstants.kArmRestSetpoint).andThen(
                        new SetElevatorPosition(Constants.ScoringConstants.Stow.kElevatorPos)
                                .finallyDo(() -> {
                                    ScoreState = ScoringPosition.Stow;
                                }));
            }

        }, Set.of(Arm.getInstance(), Elevator.getInstance()));

    }

    public static Command SourceIntake() {
        return new SetElevatorPosition(Constants.ScoringConstants.Source.kElevatorPos)
                .andThen(new SetArmPosition(Constants.ScoringConstants.Source.kArmSourcePosition)
                        .finallyDo(() -> ScoreState = ScoringPosition.Source));
    }

    public static Command SchloopCommand() {
        return new SetArmPosition(Constants.ScoringConstants.Stow.kArmStowPos).andThen(
                new SetElevatorPosition(Constants.ScoringConstants.Schloop.kElevatorPos).finallyDo(() -> {
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