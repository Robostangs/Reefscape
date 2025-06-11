package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class Deploy extends Command {

  Climber climber;
  boolean smart;
  double time;

  /**
   * this runs the climber at at least 107 degrees and if the timer has been going for more than 1 second
   * @param smart 
   */

  public Deploy(boolean smart) {
    climber = Climber.getInstance();
    this.addRequirements(climber);
    this.smart = smart;
  }



// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.postStatus("Deploying Climber");
    climber.setServoAngle(107);

    time = Timer.getFPGATimestamp();

  }

  @Override
  public void execute() {

    double currTime = Timer.getFPGATimestamp();
    if (climber.getServoPosition() >= 107 &&  currTime-time > 1) {
      climber.runClimber(Constants.ClimberConstants.kExtensionDutyCycle);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    climber.runClimber(0d);
    climber.postStatus("Climber Deployed");
    climber.setServoAngle(0);
    climber.setBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (smart) {
      return climber.getClimberPosition() >= Constants.ClimberConstants.kMaxExtension;

    } else {
      return false;
    }
  }

}
