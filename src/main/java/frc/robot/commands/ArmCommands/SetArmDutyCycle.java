package frc.robot.commands.ArmCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmDutyCycle extends Command {

  Arm arm;
  DoubleSupplier speed;
/**
 * this gets the input from manip and move the arm at that speed that is inputted
 * @param speed this makes the 10% what is inputted
 */
  public SetArmDutyCycle(DoubleSupplier speed) {
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


