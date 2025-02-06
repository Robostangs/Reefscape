// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Lift;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.TunerConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
            .withRotationalDeadband(
                    Constants.TunerConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond * 0.1) // Add a
                                                                                                              // 10%
                                                                                                              // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(
            Constants.TunerConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond));

    private final CommandXboxController xDrive = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);
    private final GenericHID xSim = new GenericHID(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
        if (Robot.isSimulation()) {
            configureSimBindings();
        }
    }

    private void configureBindings() {
        if (Robot.isSimulation()) {
            drivetrain.setDefaultCommand(
                    drivetrain.applyRequest(() -> drive.withVelocityX((-xSim.getRawAxis(0))
                            * Constants.TunerConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond))
                            .withVelocityY((xSim.getRawAxis(1))
                                    * Constants.TunerConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                            .in(MetersPerSecond))
                            .withRotationalRate((xSim.getRawAxis(2))
                                    * Constants.TunerConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond)));

        } else {
            drivetrain.setDefaultCommand(
                    // Drivetrain will execute this command periodically
                    drivetrain.applyRequest(() -> drive.withVelocityX((-xDrive.getLeftY())
                            * Constants.TunerConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond))
                            .withVelocityY((-xDrive.getLeftX())
                                    * Constants.TunerConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                            .in(MetersPerSecond))
                            .withRotationalRate((-xDrive.getRightX())
                                    * Constants.TunerConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond)));
        }

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))
        // ));

        // reset the field-centric heading on left bumper press
        xDrive.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureSimBindings() {

        // new Trigger(() -> m_driverControllerSim.getRawButtonPressed(1))
        // .whileTrue(new Lift(10d));
        new Trigger(() -> xSim.getRawButtonPressed(1))
                .toggleOnTrue(
                        // new MoveArm(400d)
                        new Lift(5d)
                // ScoringFactory.L1Score()
                );

        // new Trigger(() -> m_driverControllerSim.getRawButtonPressed(2)).whileTrue(
        // new Lift(60d)
        // );

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
