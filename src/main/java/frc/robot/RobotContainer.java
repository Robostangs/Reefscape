// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Lift;
import frc.robot.commands.MoveArm;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.IntakeCommands.RunIntake;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
 
  private final CommandXboxController xDrive = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final GenericHID xSim = new GenericHID(2);

  public RobotContainer() {
    // Configure the trigger bindings
    configureDriveBindings();
    if (Robot.isSimulation()) {
      configureSimBindings();
    }
  }

  private void configureDriveBindings() {

    // xDrive.x().toggleOnTrue(new Extend().withTimeout(1.5).andThen(new RunIntake(true)).finallyDo(Retract.retract()));
    xDrive.rightStick().toggleOnTrue(new Extend().andThen(new RunIntake()).finallyDo(Retract.Retract));
    xDrive.y().toggleOnTrue(new Retract());
    xDrive.x().toggleOnTrue(new Extend());



  }

  private void configureSimBindings() {

    // new Trigger(() -> m_driverControllerSim.getRawButtonPressed(1))
    // .whileTrue(new Lift(10d));
    new Trigger(() -> xSim.getRawButtonPressed(1))
        .toggleOnTrue(
            new Lift(5d)
        );

    // new Trigger(() -> m_driverControllerSim.getRawButtonPressed(2)).whileTrue(
    // new Lift(60d)
    // );

  }

}
