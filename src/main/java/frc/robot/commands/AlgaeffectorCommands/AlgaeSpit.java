package frc.robot.commands.AlgaeffectorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeffectorConstants;
import frc.robot.subsystems.Algaeffector;
import frc.robot.subsystems.Endeffector;

public class AlgaeSpit extends Command {
   Algaeffector algaeffector;

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
    @Override
    public void end(boolean interrupted){
        algaeffector.postStatus("Spitting");
        algaeffector.setEneffdector(0);
        algaeffector.setEndeffectorBrake();
    }
    @Override
    public boolean isFinished() {
        return false;
    }

}
