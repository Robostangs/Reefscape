package frc.robot.commands.AlgaeffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algaeffector;

// This command controls the Algaeffector subsystem to perform the "spit" action, which activates the end effector motor at a specified duty cycle to eject algae. 



public class AlgaeSpit extends Command {
    Algaeffector algaeffector;

    /**
     * this gets the algae out of the algae effector at a speed of 70% speed in the other way
     * No parameter (set value)
     */
    
    public AlgaeSpit(){
        algaeffector = Algaeffector.getInstance();
        addRequirements(algaeffector);
    }
    @Override
    public void initialize(){
        algaeffector.postStatus("algae spitting");
    }
    @Override
    public void execute(){
        algaeffector.setEneffdector(Constants.AlgaeffectorConstants.kAlgaeffectorDutyCyle);
    }
    // This method is called when the command ends or is interrupted. It will set the endeffector to zero which will stop it.
    @Override
    public void end(boolean interrupted){
        algaeffector.postStatus("Spitting");
        algaeffector.setEneffdector(0);
        algaeffector.setEndeffectorBrake();
    }
    //returns true when the command should end
    @Override
    public boolean isFinished() {
        return false;
    }

}
