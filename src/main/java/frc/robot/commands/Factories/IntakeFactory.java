package frc.robot.commands.Factories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.IntakeCommands.RunIntake;

public class IntakeFactory {

    public static Command IntakeCoral(){
        return new Extend().alongWith(new RunIntake()).finallyDo(Retract.Retract);
    }

    public static Command Schloop(){
        return new Extend().alongWith(new RunIntake()).andThen(new Retract())
        .finallyDo(ScoringFactory.returnHome());
    } 
    
    
}
