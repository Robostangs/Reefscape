package frc.robot.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
public class Algaeintake extends Command {


//PStart - P2L - P1R - P1L

  IntakePivot intake;
/**
 * A command the sets the intake position be horizontally out 
 */
  public Algaeintake() {
    intake = IntakePivot.getInstance();
    this.addRequirements(intake);

  }

  @Override
  public void initialize() {
    intake.postStatus("Going to puke");
    intake.setAlgaeIntakePosition();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    intake.postStatus("Puked");
    intake.setIntakePivotBrake();
    intake.stopBar();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}

