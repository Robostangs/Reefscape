package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWheels;

/**
 * Command to reverse the intake wheels and eject coral from the robot.
 * When scheduled, this command sets the intake wheels to run at a negative duty cycle,
 * which pushs the coral out of the intake if needed. The command continues running until interrupted,
 * then it stops the intake and updates the status.
 */
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
