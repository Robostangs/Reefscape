package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class Reel extends Command {

  Climber climber;

  public Reel() {
    climber = Climber.getInstance();
    this.addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.postStatus("Retracting Climber");

  }

  @Override
  public void execute() {
    if (climber.getClimberPosition() > Constants.ClimberConstants.kReelSafe) {
      climber.runClimber(Constants.ClimberConstants.kReelDutyCycle);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    climber.runClimber(0d);
    climber.postStatus("Climber Deployed");
    climber.setBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return climber.getClimberPosition() <= Constants.ClimberConstants.kReelSafe;
  }

}
