package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;

public class Extend extends Command {

  IntakePivot intake;
  IntakeWheels aintake;

  public Extend() {
    intake = IntakePivot.getInstance();
    aintake = IntakeWheels.getInstance();
    this.addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.postStatus("DEPLOY DEPLOY DEPLOY");

    intake.setExtendPosition();

  }

  @Override
  public void execute() {
    aintake.runDutyCycleIntake(0.75);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.postStatus("Retracted");
    aintake.runDutyCycleIntake(0);
    // intake.setIntakePiviotBrake();
    intake.stopBar();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO do this when we put it on
    return false;
    // return intake.isIntakeatSetpoint(true);
  }

}
