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
   * A command that connects the servo to the rachet then deploys the climber onto the hanging cage
   * @param smart whether to stop when it reaches the setpoint or not
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
    climber.setServoAngle(Constants.ClimberConstants.servoRatchetPosition);

    time = Timer.getFPGATimestamp();

  }

  @Override
  public void execute() {

    double currTime = Timer.getFPGATimestamp();
    if (climber.getServoPosition() >= Constants.ClimberConstants.servoRatchetPosition &&  currTime-time > Constants.ClimberConstants.timeToRatchet) {
      climber.runClimber(Constants.ClimberConstants.kExtensionDutyCycle);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    climber.runClimber(0d);
    climber.postStatus("Climber Deployed");
    climber.setServoAngle(0);
    
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
