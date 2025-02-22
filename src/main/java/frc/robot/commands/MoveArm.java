package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArm extends Command {
  Arm arm;
  double rotations;

  public MoveArm(double rotations) {

    this.rotations = rotations;
    arm = Arm.getInstance();
    addRequirements(arm);

  }

  @Override
  public void initialize() {
    arm.setArmPosition(rotations);
    arm.postStatus("going to this rotation:" + rotations);
  }

  @Override
  public void execute() {
    arm.setArmMotionMagic();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.postStatus("at this rotation:" + rotations);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isArmAtTarget();
  }

}
