package frc.robot.commands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorDutyCycle extends Command {

  /*
   * This reads the input from the manip controller and sets the elevator to that
   * The elevator is first said to be in state of manual adjustment 
   * Then gets the input from the joystick constantly
   * When the code ends, the elevator moves at {@code elevatorDutyCycle} speed and its status is stopped
   */

  Elevator elevator;
  DoubleSupplier speed;

  public SetElevatorDutyCycle(DoubleSupplier speed) {
    elevator = Elevator.getInstance();
    addRequirements(elevator);
    this.speed = speed;
  }

  @Override
  public void initialize() {
    elevator.postStatus("Manually Adjusting Elevator");
  }

  @Override
  public void execute() {
    elevator.setElevatorDutyCycle(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
      elevator.setElevatorDutyCycle(0.03);
      
    
    elevator.postStatus("Elevator Stopped");

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
