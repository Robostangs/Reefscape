package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class Reel extends Command {

  Climber climber;
  boolean smart;

  /**
   * A command that reels the climber in
   * @param smart whether to stop when it reaches the setpoint or not
   */
  public Reel(boolean smart) {
    climber = Climber.getInstance();
    this.addRequirements(climber);
    this.smart = smart;
  }

  @Override
  public void initialize() {
    climber.postStatus("Retracting Climber");
  }

  @Override
  public void execute() {

      climber.runClimber(Constants.ClimberConstants.kReelDutyCycle);
    
  }

  @Override
  public void end(boolean interrupted) {

    climber.runClimber(0d);
    climber.postStatus("Climber Reeled ");
    climber.setBrake();
  }

  @Override
  public boolean isFinished() {
    if (smart) {
      return (climber.getClimberPosition()) <= Constants.ClimberConstants.kReelSafe;
    } else {
      return false;
    }
  }

}
