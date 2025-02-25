package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class Extend extends Command {

  IntakePivot intake;

  public Extend() {
    intake = IntakePivot.getInstance();
    this.addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.postStatus("DEPLOY DEPLOY DEPLOY");
    intake.setPivotZero();

    intake.setExtendPosition();

  }

  @Override
  public void execute() {
    intake.runIntakeMotionMagic();

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
    // TODO do this when we put it on
    // return false;[\]
    return intake.isIntakeatSetpoint(true);
  }

}
