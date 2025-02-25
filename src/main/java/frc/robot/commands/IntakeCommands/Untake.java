package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWheels;

public class Untake extends Command{

    IntakeWheels intake;

    public Untake() {
        intake = IntakeWheels.getInstance();
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        intake.postStatus("Untaking");
    }

    @Override
    public void execute() {

        intake.runIntake(-0.75);

    }

    @Override
    public void end(boolean interrupted) {
        intake.postStatus("Coral Out");
        intake.runIntake(0d);

    }

  @Override
  public boolean isFinished() {
    return false;
  }

}
