package frc.robot.commands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RunElevator extends Command {
  Elevator elevator;
  DoubleSupplier speed;

  public RunElevator(DoubleSupplier speed) {
    elevator = Elevator.getInstance();
    addRequirements(elevator);
    this.speed = speed;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevator.setElevatorDutyCycle(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setElevatorDutyCycle(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
    
}
