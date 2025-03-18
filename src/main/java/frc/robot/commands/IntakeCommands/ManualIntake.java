package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class ManualIntake extends Command {
    
    IntakePivot intake;
    DoubleSupplier dutycycleIntake;
    public ManualIntake(DoubleSupplier dutycycleIntake) {
        intake = IntakePivot.getInstance();
        this.dutycycleIntake = dutycycleIntake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.postStatus("intake manny movieeee");
    }

    @Override
    public void execute() {
        intake.setPiviotDutyCycle(dutycycleIntake.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPiviotDutyCycle(0);
        intake.postStatus("intake not movieeee");


        
    }

    public boolean isFinished() {
        return false;
    }

    
}
