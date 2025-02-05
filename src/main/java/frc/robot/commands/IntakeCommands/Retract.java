package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Lift;
import frc.robot.commands.MoveArm;
import frc.robot.subsystems.Intake;

public class Retract extends Command {

  Intake intake;

  public Retract() {
    intake = Intake.getInstance();
    addRequirements(intake);
  }

  public static Runnable retract() {
    return () -> {
      new Retract().withTimeout(0.3);
    };
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.postStatus("FALLING BACK");

  }

  @Override
  public void execute() {
    intake.retractBar();

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
        //TODO do this when we put it on
    // return intake.getIntakeSensor();
    return false;
  }

}
