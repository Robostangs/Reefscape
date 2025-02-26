package frc.robot.commands.Factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.MoveArm;
import frc.robot.commands.ElevatorCommands.Lift;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.subsystems.Elevator;

public class ScoringFactory {

    public static Command L1Position() {
        return new Lift(Constants.ScoringConstants.L1.kElevatorStart)
                .andThen(new MoveArm(Constants.ScoringConstants.L1.kArmScoringPosition))
                // .andThen(
                // new WaitUntilCommand(
                // () -> Arm.getInstance().getTargetArmAngle() >
                // Constants.ScoringConstants.L1.kArmSafePosition)
                .andThen(new Lift(Constants.ScoringConstants.L1.kElevatorEnd));
    }

    public static Command L2Position() {
        return new Lift(Constants.ScoringConstants.L2.kElevatorStart).andThen(
                new MoveArm(Constants.ScoringConstants.L2.kArmScoringPosition))
                // .alongWith(new WaitUntilCommand(
                // () -> Arm.getInstance().getTargetArmAngle() >
                // Constants.ScoringConstants.L2.kArmSafePosition)
                .andThen(new Lift(Constants.ScoringConstants.L2.kElevatorEnd));
    }

    public static Command L3Position() {
        return new Lift(Constants.ScoringConstants.L3.kElevatorPos)
                .alongWith(new MoveArm(Constants.ScoringConstants.L3.kArmScoringPosition));
    }

    public static Command L4Position() {
        return new Lift(Constants.ScoringConstants.L4.kElevatorPos)
                .alongWith(
                        new WaitUntilCommand(
                                () -> Elevator.getInstance()
                                        .getElevatorPositionMeters() > Constants.ElevatorConstants.kHomePosition)
                                .andThen(
                                        new MoveArm(Constants.ScoringConstants.L4.kArmScoringPosition)));
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

    public static Command returnHome() {
        return new MoveArm(Constants.ArmConstants.kArmRestSetpoint).andThen(
                new Lift(Constants.ScoringConstants.Schloop.kElevatorPos));
    }

    public static Command getCoralCommand() {
        return new MoveArm(Constants.ArmConstants.kArmRestSetpoint).andThen(
                new Lift(0.66));

    }

    public static Runnable returnHomeRun() {
        return () -> {
            new MoveArm(Constants.ArmConstants.kArmRestSetpoint).andThen(
                    // TODO make this a constant
                    new Lift(Constants.ScoringConstants.Schloop.kElevatorPos));
        };
    }

    public static Runnable getCoral() {
        return () -> {
            new MoveArm(Constants.ArmConstants.kArmRestSetpoint).andThen(
                    // TODO make this a constant
                    new Lift(0.66));
        };
    }
}