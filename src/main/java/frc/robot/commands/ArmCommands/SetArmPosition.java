package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class SetArmPosition extends Command {
  Arm arm;
  double angle;

/**
 * A command that sets the arm to a target angle
 * Motion magic is used to move the arm to the target angle smoothly
 * 
 * @param angle the setpoint for the arm
 */
  public SetArmPosition(double angle) {

    this.angle = angle;
    arm = Arm.getInstance();
    addRequirements(arm);

  }
  // Start of the command, sets the arm to the setpoint
  @Override
  public void initialize() {
    arm.setArmPosition(angle);
    arm.postStatus("Arm going to this angle:" + angle);
  }
  
  @Override
  public void execute() {
    arm.setArmMotionMagic();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.postStatus("Arm at this rotation:" + angle);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isArmAtTarget();
  }

}
