package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Extend extends Command {

    Intake intake;

    public Extend() {
        intake = Intake.getInstance();
        addRequirements(intake);
    }

   

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    intake.extendBar();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}


