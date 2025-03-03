package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class Heimlich extends Command {

  IntakePivot intake;

  public Heimlich() {
    intake = IntakePivot.getInstance();
    this.addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.postStatus("Going to puke");
    intake.setHeimlichPosition();

  }

  @Override
  public void execute() {

    intake.runIntakeMotionMagic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.postStatus("Puked");
    intake.setIntakePivotBrake();
    intake.stopBar();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}