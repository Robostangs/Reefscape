package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;

public class HomeIntake extends Command{

    IntakePivot intake;
    /**
     * A command that runs the intake at a duty cycle and then when it ends sets the setpoint to the hardstop
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
