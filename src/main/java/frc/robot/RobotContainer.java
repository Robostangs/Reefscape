// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Lift;
import frc.robot.commands.MoveArm;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final XboxControllerSim m_driverControllerSim = new XboxControllerSim(1);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    if (Robot.isSimulation()) {
      configureSimBindings();
    }
  }

  private void configureBindings() {

    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.1)
        .whileTrue(new Lift(() -> m_driverController.getRightTriggerAxis()));

  }

  private void configureSimBindings() {
    new Trigger(() -> m_driverControllerSim.getOutput(1)).toggleOnTrue(new PrintCommand("you aren't tweaking"));
  }

}
