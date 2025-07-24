package frc.robot.commands.ClimberCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ManualAdjustClimber extends Command {

    Climber climber;
    DoubleSupplier dutyCycle;
    double time;
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
        climber.setServoAngle(Constants.ClimberConstants.servoRatchetPosition);
        time = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        
        double currTime = Timer.getFPGATimestamp();
        if (climber.getServoPosition() >= Constants.ClimberConstants.servoRatchetPosition &&  currTime-time > Constants.ClimberConstants.timeToRatchet) {
            climber.runClimber(dutyCycle.getAsDouble());
        }


    }

    @Override
    public void end(boolean interrupted) {

        climber.runClimber(0);
    }

}
