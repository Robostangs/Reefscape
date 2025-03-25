package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;

public class Retract extends Command {

  IntakePivot intake;
  IntakeWheels intakeWheels;

  /**
   * A command that sets the position of the intake to the retracted setpoint which is out of the way of the arm
   */
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
  public void execute() {}

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
