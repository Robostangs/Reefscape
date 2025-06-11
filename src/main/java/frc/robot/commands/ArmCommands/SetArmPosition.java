package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmPosition extends Command {
  Arm arm;
  double rotations;

  /**
   * sets the rotations for Motion Magic to be reached to
   * @param rotations has a set position of where Motion Magic should be
   */

  public SetArmPosition(double rotations) {

    this.rotations = rotations;
    arm = Arm.getInstance();
    addRequirements(arm);

  }

  @Override
  public void initialize() {
    arm.setArmPosition(rotations);
    arm.postStatus("Arm going to this rotation:" + rotations);
  }

  @Override
  public void execute() {
    arm.setArmMotionMagic();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.postStatus("Arm at this rotation:" + rotations);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isArmAtTarget();
  }

}
