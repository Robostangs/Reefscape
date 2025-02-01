package frc.robot.commands.EndeffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Endeffector;

public class Spit extends Command {
    Endeffector endeffector;

    public Spit() {
        endeffector = Endeffector.getInstance();
        addRequirements(endeffector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        endeffector.setEneffector(Constants.EndefectorConstants.kEndeffectorSpit);
    }

    @Override
    public void end(boolean interrupted) {

        endeffector.setEneffector(0);
    }

    @Override
    public boolean isFinished() {
        return endeffector.getEndefectorSensor();
    }
}
