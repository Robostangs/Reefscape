package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;

public class Extend extends Command {

  IntakePivot intake;
  IntakeWheels intakeWheels;
  boolean auto;

  /**
   * A command that sets the position of the intake to extended
   */
  public Extend(boolean auto) {
    intake = IntakePivot.getInstance();
    intakeWheels = IntakeWheels.getInstance();
    this.auto = auto;
    this.addRequirements(intake);
  }

  @Override
  public void initialize() {

    intake.postStatus("DEPLOY DEPLOY DEPLOY");
    intake.setExtendPosition();
    if (auto) {
      intakeWheels.runDutyCycleIntake(Constants.IntakeConstants.kIntakeSpeed);
    }

  }

  @Override
  public void execute() {
  }

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
