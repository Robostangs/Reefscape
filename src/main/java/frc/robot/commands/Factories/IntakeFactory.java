package frc.robot.commands.Factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.MoveArm;
import frc.robot.commands.ElevatorCommands.Lift;
import frc.robot.commands.EndeffectorCommands.Slurp;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.IntakeCommands.RunIntake;

public class IntakeFactory {

    public static Command IntakeCoral() {
        return new Extend().andThen(new RunIntake()).finallyDo(Retract.Retract);
    }

    public static Command Schloop() {
        return new Extend()
                .alongWith(new RunIntake()).andThen(new Retract())
                .finallyDo(ScoringFactory.getCoral());
    }

    public static Command SourceIntake() {
        return new Lift(Constants.ScoringConstants.Source.kElevatorPos).alongWith(
                new MoveArm(Constants.ScoringConstants.Source.kArmSourcePosition))
                .alongWith(new Slurp());
    }

    // public static Command Climb(){}

}
