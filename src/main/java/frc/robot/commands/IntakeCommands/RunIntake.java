package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {

    Intake intake;
    // Endeffector endeffector;

    public RunIntake() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

   

    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.postStatus("SCHLOOPING");
  }

  @Override
  public void execute() {

    intake.runIntake(0.5);

    // endeffector.setEneffdector(0.3);

  }

  @Override
  public void end(boolean interrupted) {
    intake.postStatus("SCHLOOPED");
    intake.runIntake(0d);


  }

  @Override
  public boolean isFinished() {
    //TODO do this when we put it on
    // return intake.getIntakeSensor();
    return false;
  }

}


