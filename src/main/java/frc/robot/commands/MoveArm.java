package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArm extends Command {
  Arm arm;
  double angle;

  public MoveArm(double angle) {

    this.angle = angle;
    arm = Arm.getInstance();
    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmMotor(angle);
    arm.postStatus("going to this angle:" + angle);
  }

  @Override
  public void execute() {
    arm.setArmMotionMagic();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.postStatus("at this angle:" + angle);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
