package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class Extend extends Command {

  IntakePivot intake;

  /**
   * A command that sets the position of the intake to extended
   */
  public Extend() {
    intake = IntakePivot.getInstance();
    this.addRequirements(intake);
  }

  @Override
  public void initialize() {

    intake.postStatus("DEPLOY DEPLOY DEPLOY");
    intake.setExtendPosition();

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intake.postStatus("Retracted");
    intake.stopBar();

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
