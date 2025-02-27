
package frc.robot.commands.EndeffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Endeffector;

public class Spit extends Command {
    Endeffector endeffector;

    /**
     * A command that spits out the pipe
     */
    public Spit() {
        endeffector = Endeffector.getInstance();
        addRequirements(endeffector);
    }

    @Override
    public void initialize() {
        endeffector.postStatus("Hawk Tuah");

    }

    @Override
    public void execute() {
        endeffector.setEneffdector(Constants.EndeffectorConstants.kEndeffectorSpit);
    }



    @Override
    public void end(boolean interrupted) {

        endeffector.postStatus("Spitted on that thang");
        endeffector.setEneffdector(0);
        endeffector.setEndeffectorBrake();
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
