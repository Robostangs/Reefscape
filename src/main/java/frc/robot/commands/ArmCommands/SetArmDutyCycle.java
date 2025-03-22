package frc.robot.commands.ArmCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmDutyCycle extends Command {

  Arm arm;
  DoubleSupplier speed;

  /**
   * A command that runs the arm at a duty cycle
   * @param speed the percent to run the arm at
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


