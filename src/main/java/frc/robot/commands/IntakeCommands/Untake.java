package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWheels;

public class Untake extends Command{

    IntakeWheels intake;

    /**
     * A command the sets the intake to a negative duty cycle to get coral out
     */
    public Untake() {
        intake = IntakeWheels.getInstance();
        addRequirements(intake);
    }

    @Override
    public void initialize() {

        intake.postStatus("Untaking");
        intake.runDutyCycleIntake(-0.75);

    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intake.postStatus("Coral Out");
        intake.runDutyCycleIntake(0d);

    }

  @Override
  public boolean isFinished() {
    return false;
  }

}
