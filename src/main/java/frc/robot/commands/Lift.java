// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
    elevator.setElevatorPosition(position);

  }

  @Override
  public void execute() {

    elevator.postStatus("elevator going to:"+ position );

  }

  @Override
  public void end(boolean interrupted) {
    elevator.postStatus("elevator done" );

  }

  @Override
  public boolean isFinished() {
    // return false;
     return elevator.getIsElevatorAtTarget();
  }
}
