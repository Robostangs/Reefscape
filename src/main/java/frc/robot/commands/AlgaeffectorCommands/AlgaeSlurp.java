package frc.robot.commands.AlgaeffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algaeffector;

// This command controls the Algaeffector subsystem to perform the "slurp" action, which activates the endeffector motor at a specified duty cycle to suck in the algae.

public class AlgaeSlurp extends Command{
    Algaeffector Algaeslurpeffector;

    /**
     * this gets the algae from the endefactor in the storing spot at 70% power
     * No paramter (set value)
     */

    public AlgaeSlurp(){
        Algaeslurpeffector = Algaeffector.getInstance();
        addRequirements(Algaeslurpeffector);
    }
    @Override
    public void initialize(){
        Algaeslurpeffector.postStatus("Algae getting slurpped");
    }
    @Override
    public void execute(){
        Algaeslurpeffector.setEneffdector(Constants.AlgaeffectorConstants.kAlgaeffectorSlurpCycle);
    }
    // This method is called when the command ends or is interrupted. It will set the endeffector to zero which will stop it.
    @Override
    public void end(boolean interrupted){
        Algaeslurpeffector.postStatus("Slurp");
        Algaeslurpeffector.setEneffdector(0);
        Algaeslurpeffector.setEndeffectorBrake();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}   
