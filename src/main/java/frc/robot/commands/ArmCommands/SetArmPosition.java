package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class SetArmPosition extends Command {
  Arm arm;
  double rotations;

/**
 * A command that sets the arm to a target rotation
 * 
 * @param rotations the target rotation for motion magic
 */
  public SetArmPosition(double rotations) {

    this.rotations = rotations;
    arm = Arm.getInstance();
    addRequirements(arm);

  }
  // Start of the command, sets the arm to the setpoint
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

  // Returns true when the arm is within the tolerence 
  @Override
  public boolean isFinished() {
    return arm.isArmAtTarget(0.01);
  }

}
