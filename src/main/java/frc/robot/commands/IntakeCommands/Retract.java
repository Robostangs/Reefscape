package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Retract extends Command {

  Intake intake;

  public Retract() {
    intake = Intake.getInstance();
    this.addRequirements(intake);
  }
  public static Runnable Retract = () -> {
    Intake intake = Intake.getInstance();
    intake.retractBar();
  };

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.postStatus("FALLING BACK");

    intake.retractBar();

  }

  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.postStatus("RETRACTED");
    intake.setIntakePiviotBrake();
    intake.stopBar();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO do this when we put it on
    // return intake.isIntakeatSetpoint(false);
    return false;
  }

}
