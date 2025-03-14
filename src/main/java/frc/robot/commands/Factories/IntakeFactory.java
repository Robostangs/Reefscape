package frc.robot.commands.Factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.MoveArm;
import frc.robot.commands.ElevatorCommands.Lift;
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
        return new Extend().alongWith(new RunIntake()
        ).andThen(new RunIntake())
        // .finallyDo(Retract.Retract)
        ;
    }

    /**
     * @return A command that {@code IntakeCoral} then moves the arm and elevator to
     *         get it
     */
    public static Command Schloop() {
        return new Extend()
                .alongWith(new RunIntake()).andThen(new Retract())
                .finallyDo(ScoringFactory.getSchloop());
    }

    /**
     * @return A command that intakes from human player station
     */
    public static Command SourceIntake() {
        return new Lift(Constants.ScoringConstants.Source.kElevatorPos).alongWith(
                new MoveArm(Constants.ScoringConstants.Source.kArmSourcePosition))
                .alongWith(new Slurp());
    }


    public static Command Vomit(){
        return new Heimlich().alongWith(new Untake());
    }

}
