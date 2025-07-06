package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;

public class HomeIntake extends Command{

    IntakePivot intake;
    /**
     * Command to home the intake by running it at a set duty cycle.
     * When finished, it sets the intake to the hardstop position.
     */
    public HomeIntake() {
        intake = IntakePivot.getInstance();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.postStatus("intake homing");
    }

    @Override
    public void execute() {
        intake.setPiviotDutyCycle(Constants.IntakeConstants.kIntakeHomeDutyCycle);

    }

    @Override
    public void end(boolean interrupted) {
        intake.setPiviotDutyCycle(0);
        intake.point3Intake();
        intake.postStatus("intake homed");


        
    }

    public boolean isFinished() {
        return false;
    }

    
}
