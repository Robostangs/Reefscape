// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Lift;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

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
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX((-xDrive.getLeftY()) * MaxSpeed) // Drive
                                                                                                   // forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY((-xDrive.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate((-xDrive.getRightX()) * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

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
