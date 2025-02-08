package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Extend extends Command {

    Intake intake;

    public Extend() {
        intake = Intake.getInstance();
        this.addRequirements(intake);
      }

   

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.postStatus("Extending");
    intake.extendBar();

  }

  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.postStatus("Retracted");
    // intake.setIntakePiviotBrake();
    intake.stopBar();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        //TODO do this when we put it on
    return intake.isIntakeatSetpoint(true);
    // return false;
  }

}


