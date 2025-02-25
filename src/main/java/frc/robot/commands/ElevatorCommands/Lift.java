// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Lift extends Command {
  Elevator elevator;
  double position;

  /** Creates a new Lift. */
  public Lift(double position) {
    elevator = Elevator.getInstance();
    addRequirements(elevator);
    this.position = position;
  }

  @Override
  public void initialize() {
    elevator.setElevatorPositionMeters(position);
    elevator.postStatus("elevator going to:" + position);

  }

  @Override
  public void execute() {

    elevator.setElevatorMotionMagic();

  }

  @Override
  public void end(boolean interrupted) {
    elevator.postStatus("elevator done");

  }

  @Override
  public boolean isFinished() {
    if (Robot.isSimulation()) {
      return false;
    } else {
      return elevator.isElevatorAtTarget();
    }
  }
}
