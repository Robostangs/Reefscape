package frc.robot.commands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RunElevator extends Command {
  Elevator elevator;
  DoubleSupplier speed;
  /**
   * A command that runs the elevator at a duty cycle
   * @param speed the percent to run the elevator at
   */
  public RunElevator(DoubleSupplier speed) {
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
      elevator.setElevatorDutyCycle(0.0325);
    
    elevator.postStatus("Elevator Stopped");

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
