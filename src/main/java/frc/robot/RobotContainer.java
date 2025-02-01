// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Lift;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final GenericHID m_driverControllerSim = new GenericHID(2);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    if (Robot.isSimulation()) {
      configureSimBindings();
    }
  }

  private void configureBindings() {

  }

  private void configureSimBindings() {

    // new Trigger(() -> m_driverControllerSim.getRawButtonPressed(1))
    //     .whileTrue(new Lift(10d));
    new Trigger(() -> m_driverControllerSim.getRawButtonPressed(1))
    .toggleOnTrue(
      new Lift(2d)
      // ScoringFactory.L1Score()
      );

    // new Trigger(() -> m_driverControllerSim.getRawButtonPressed(2)).whileTrue(
    // new Lift(60d)
    // );

  }

}
