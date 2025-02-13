package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class HomeIntake extends Command{

    Intake intake;
    public HomeIntake() {
        intake = Intake.getInstance();

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
        intake.setPiviotZero();
        intake.postStatus("intake homed");


        
    }

    public boolean isFinished() {
        // return elevator.isElevatorAtHome();
        return false;
    }

    
}
