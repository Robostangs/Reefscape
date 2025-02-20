package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;


import edu.wpi.first.wpilibj2.command.Command;

public class RunArm extends Command {

  Arm arm;
  DoubleSupplier speed;
  boolean zero;

  public RunArm(DoubleSupplier speed, boolean zero) {
    arm = Arm.getInstance();
    addRequirements(arm);
    this.speed = speed;
    this.zero = zero;
  }

  @Override
  public void initialize() {
    arm.postStatus("Manually Adjusting Arm");
    arm.setBrakeMode();
  }

  @Override
  public void execute() {
    arm.setArmDutyCycle(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    arm.setArmDutyCycle(0);
    arm.postStatus("Elevator Stopped");
    arm.setBrakeMode();


  }

  @Override
  public boolean isFinished() {
    return false;
  }
    
}


