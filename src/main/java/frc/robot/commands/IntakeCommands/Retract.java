package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class Retract extends Command {

  IntakePivot intake;

  public Retract() {
    intake = IntakePivot.getInstance();
    this.addRequirements(intake);

  }
  public static Runnable Retract = () -> {
    IntakePivot intake = IntakePivot.getInstance();
    intake.setRetractPosition();
  };

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.postStatus("FALLING BACK");
    intake.setRetractPosition();

  }

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.postStatus("Retracted");
    intake.setIntakePivotBrake();
    intake.stopBar();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
