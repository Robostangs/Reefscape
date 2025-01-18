// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Lift extends Command {
  Elevator elevator;
  double position;

  /** Creates a new Lift. */
  public Lift(DoubleSupplier position) {
    elevator = Elevator.getInstance();
    addRequirements(elevator);
    this.position = position.getAsDouble();
  }


  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    elevator.setElevatorPosition(position);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
