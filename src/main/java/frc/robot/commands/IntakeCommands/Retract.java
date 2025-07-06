package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;

/**

 * This command moves the intake pivot to the retracted setpoint, stops the intake wheels,
 * and applies the pivot brake. It is intended to keep the intake clear of the arm and
 * out of the way during other operations.
 */
public class Retract extends Command {

  IntakePivot intake;
  IntakeWheels intakeWheels;


  public Retract() {
    intake = IntakePivot.getInstance();
    intakeWheels = IntakeWheels.getInstance();
    this.addRequirements(intake);

  }
  public static Runnable Retract = () -> {
    IntakePivot intake = IntakePivot.getInstance();
    intake.setRetractPosition();
  };

  @Override
  public void initialize() {
    intake.postStatus("FALLING BACK");
    intake.setRetractPosition();
    intakeWheels.stopIntake();

    
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    intake.postStatus("Retracted");
    intake.setIntakePivotBrake();
    intake.stopBar();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
