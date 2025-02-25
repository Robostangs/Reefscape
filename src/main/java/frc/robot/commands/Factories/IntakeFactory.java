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
        return new Extend().alongWith(new RunIntake()).finallyDo(Retract.Retract);
    }

    public static Command Schloop() {
        return new Extend()
                .andThen(new RunIntake()).andThen(new Retract())
                .finallyDo(ScoringFactory.returnHomeRun());
    }

    public static Command SourceIntake() {
        return new MoveArm(Constants.ArmConstants.kArmHumanPlayer)
                .andThen(new Slurp().withTimeout(Constants.SwerveConstants.AutoConstants.kSlurpTimeout));
    }

}
