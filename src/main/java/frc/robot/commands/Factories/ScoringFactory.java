package frc.robot.commands.Factories;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.MoveArm;
import frc.robot.commands.ElevatorCommands.Lift;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;

public class ScoringFactory {

    public static enum ScoringPosition {
        L1, L2, L3, L4, Stow, Schloop
    }

    static Alert stowAlert = new Alert("Can't go to stow", Alert.AlertType.kError);

    public static ScoringPosition ScoreState = ScoringPosition.Stow;

    public static Command L1Position() {
        return new Lift(Constants.ScoringConstants.L1.kElevatorStart)
                .andThen(new MoveArm(Constants.ScoringConstants.L1.kArmScoringPosition))
                .andThen(new Lift(Constants.ScoringConstants.L1.kElevatorEnd));

    }

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
                                .andThen(
                                        new MoveArm(Constants.ScoringConstants.L4.kArmScoringPosition))
                                .finallyDo(() -> {
                                    ScoreState = ScoringPosition.L4;
                                }));
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

    public static Command Stow(){


        return new MoveArm(Constants.ScoringConstants.Stow.kArmStowPos).andThen(
                            new Lift(Constants.ScoringConstants.Stow.kElevatorPos));

        // return new DeferredCommand(()->{
        //     if (ScoreState.equals(ScoringPosition.L2)) {
        //         return StowL2();
        //     } else if (ScoreState.equals( ScoringPosition.L3) || ScoreState.equals( ScoringPosition.L4)) {
        //         return new MoveArm(Constants.ArmConstants.kArmRestSetpoint).andThen(
        //                 new Lift(Constants.ScoringConstants.Stow.kElevatorPos));
        //     } else {
        //         stowAlert.set(true);
        //         return new PrintCommand("Can't stow");
        //     }
        // }, Set.of(Arm.getInstance(),Elevator.getInstance()));


    }

    public static Command SourceIntake(){
        return new Lift(Constants.ScoringConstants.Source.kElevatorPos)
        .andThen( new MoveArm(Constants.ScoringConstants.Source.kArmSourcePosition));
    }

    public static Command StowL2() {
        return new Lift(Constants.ScoringConstants.Stow.kElevatorPos)
                .andThen(new MoveArm(Constants.ScoringConstants.Stow.kArmStowPos));
    }

    public static Command SchloopCommand() {
        return new MoveArm(Constants.ScoringConstants.Stow.kArmStowPos).andThen(
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

    public static Runnable getSchloop() {
        return () -> {
            new MoveArm(Constants.ArmConstants.kArmRestSetpoint).andThen(
                    new Lift(Constants.ScoringConstants.Source.kElevatorPos));
        };
    }
}