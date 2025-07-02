// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorPosition extends Command {

  /*
   * This allows the user to have the elevator go to a set point
   * It starts with the goal position being set 
   * Then it uses motion magic during the running of the code to effectively reach the target spot
   * The elevator stops once it has reached its position
   */

  Elevator elevator;
  double position;

  public SetElevatorPosition(double position) {
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

      return elevator.isElevatorAtTarget();
    }
  }

