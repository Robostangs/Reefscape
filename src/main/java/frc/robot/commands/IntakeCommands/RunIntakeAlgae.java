package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeWheels;


 
public class RunIntakeAlgae extends Command {

    IntakeWheels intake;

    /**
     * A command that runs the intake wheels to intake the coral
     */
    public RunIntakeAlgae() {
        intake = IntakeWheels.getInstance();
        addRequirements(intake);
    }
    
  public static Runnable AlgaeFF = () -> {
      IntakeWheels.getInstance().runDutyCycleIntake(Constants.IntakeConstants.kAlgaeFF);
  };
  @Override
  public void initialize() {

    intake.postStatus("Intaking Algae");
    intake.runDutyCycleIntake(Constants.IntakeConstants.kAlgaeIntake);

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intake.postStatus("Algae In");
    intake.runDutyCycleIntake(0d);


  }

  @Override
  public boolean isFinished() {
      return false;
 
  }

}


