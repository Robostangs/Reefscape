package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArm extends Command {
  Arm arm;
  double angle;

  public MoveArm(DoubleSupplier angle) {

    this.angle = angle.getAsDouble();
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  public MoveArm() {

    this.angle = -25;
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setSimArmMotor(new Rotation2d(angle));

  }

  @Override
  public void execute() {
    arm.postStatus("going to this angle:" + angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
