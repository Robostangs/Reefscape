package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class Deploy extends Command {

  Climber climber;
  boolean smart;

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
  }

  @Override
  public void execute() {

    if(climber.getServoPosition() >= 107)
    climber.runClimber(Constants.ClimberConstants.kExtentionDutyCycle);

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
      return climber.getClimberPosition() >= Constants.ClimberConstants.kSafeDeployExtention;

    } else {
      return false;
    }
  }

}
