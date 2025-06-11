package frc.robot.commands.ClimberCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ManualAdjustClimber extends Command {

    Climber climber;
    DoubleSupplier dutyCycle;
    
    /**
     * this deploys the climber at what ever speed is inputted by the controller at 50% speed
     * @param dutyCycle this makes the speed of the input 50%
     */

    public ManualAdjustClimber(DoubleSupplier dutyCycle) {
        climber = Climber.getInstance();
        this.dutyCycle = dutyCycle;

    }

    @Override
    public void initialize() {
        climber.postStatus("Adjusting Climber");
    }

    @Override
    public void execute() {
        climber.runClimber(dutyCycle.getAsDouble());

    }

    @Override
    public void end(boolean interrupted) {

        climber.runClimber(0);
    }

}
