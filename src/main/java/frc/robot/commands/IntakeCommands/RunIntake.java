package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeWheels;

public class RunIntake extends Command {

    IntakeWheels intake;

    public RunIntake() {
        intake = IntakeWheels.getInstance();
        addRequirements(intake);
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.postStatus("Intaking");
    intake.runDutyCycleIntake(0.65);

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


