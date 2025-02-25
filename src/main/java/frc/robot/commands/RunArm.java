package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class RunArm extends Command {

  Arm arm;
  DoubleSupplier speed;

  public RunArm(DoubleSupplier speed) {
    arm = Arm.getInstance();
    addRequirements(arm);
    this.speed = speed;
  }

  @Override
  public void initialize() {
    arm.postStatus("Manually Adjusting Arm");
  }

  @Override
  public void execute() {
    arm.setArmDutyCycle(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    arm.setArmDutyCycle(0);
    arm.postStatus("Elevator Stopped");


  }

  @Override
  public boolean isFinished() {
    return false;
  }
    
}


