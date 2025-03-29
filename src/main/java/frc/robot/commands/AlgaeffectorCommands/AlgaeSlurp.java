
package frc.robot.commands.AlgaeffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algaeffector;

public class AlgaeSlurp extends Command{
    Algaeffector Algaeslurpeffector;
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
