package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeWheels;

public class RunIntake extends Command {

    IntakeWheels intake;

    /**
     * A command that runs the intake wheels to intake the coral
     */
    public RunIntake() {
        intake = IntakeWheels.getInstance();
        addRequirements(intake);
    }
    
  public static Runnable AlgaeFF = () -> {
      IntakeWheels.getInstance().runDutyCycleIntake(Constants.IntakeConstants.kAlgaeFF);
  };
  public static Runnable AlgaeOut = () -> {
    IntakeWheels.getInstance().runDutyCycleIntake(Constants.IntakeConstants.kAlgaeOut);
};
  @Override
  public void initialize() {

    intake.postStatus("Intaking");
    intake.runDutyCycleIntake(Constants.IntakeConstants.kIntakeSpeed);

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intake.postStatus("Coral In");
    intake.runDutyCycleIntake(0d);


  }

  @Override
  public boolean isFinished() {
      return false;
 
  }

}


