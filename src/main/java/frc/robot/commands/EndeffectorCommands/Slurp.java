package frc.robot.commands.EndeffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Endeffector;

public class Slurp extends Command{

    Endeffector endeffector;



    /**
     * A command that intakes from the endeffector
     */
    public Slurp(){
        endeffector = Endeffector.getInstance();
        addRequirements(endeffector);
        
    }
    @Override
    public void initialize() {
        endeffector.postStatus("Slurping");
        
    }

    @Override
    public void execute() {

        endeffector.setEneffdector(Constants.EndeffectorConstants.kEndeffectorSlurp);
        
    }

    @Override
    public void end(boolean interrupted) {

        endeffector.postStatus("Slurped");
        endeffector.setEneffdector(0);


        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
