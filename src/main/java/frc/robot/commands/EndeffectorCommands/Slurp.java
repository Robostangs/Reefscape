package frc.robot.commands.EndeffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Endeffector;

public class Slurp extends Command {

    Endeffector endeffector;
    boolean L1;

    /**
     * A command that intakes from the endeffector
     */
    public Slurp(boolean L1) {
        endeffector = Endeffector.getInstance();
        this.L1 = L1;
        addRequirements(endeffector);

    }

    @Override
    public void initialize() {
        endeffector.postStatus("Slurping");

    }

    @Override
    public void execute() {

        if (!L1) {
            endeffector.setEneffdector(Constants.EndeffectorConstants.kEndeffectorSlurp);
        } else {
            endeffector.setEneffdector(Constants.EndeffectorConstants.kEndeffectorSlurp-0.2
            );

        }

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
