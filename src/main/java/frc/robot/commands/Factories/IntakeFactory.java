package frc.robot.commands.Factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.SetArmPosition;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.EndeffectorCommands.Slurp;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.Heimlich;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.Untake;

public class IntakeFactory {

    /**
     * @return A command extends intake then runs the wheels and finally retracts
     */
    public static Command IntakeCoral() {
        return new Extend().alongWith(new RunIntake()).andThen(new RunIntake())
                .finallyDo(Retract.Retract);
    }

    /**
     * @return A command that intakes from human player station
     */
    public static Command SourceIntake() {
        return new SetElevatorPosition(Constants.ScoringConstants.Source.kElevatorPos).alongWith(
                new SetArmPosition(Constants.ScoringConstants.Source.kArmSourcePosition))
                .alongWith(new Slurp());
    }

    /**
     * @return A command that extends the intake and runs the wheels
     */
    public static Command algaeNamNam() {
        return new Heimlich().alongWith(new RunIntake())
                .finallyDo(Retract.Retract);
    }

    /**
     * @return A command that extends the intake and runs the wheels
     */
    public static Command Vomit() {
        return new Heimlich().alongWith(new Untake()
                .finallyDo(Retract.Retract));
    }

}
