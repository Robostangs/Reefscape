// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.FlippingUtil;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommands.MoveArm;
import frc.robot.commands.ArmCommands.RunArm;
import frc.robot.commands.ClimberCommands.Deploy;
import frc.robot.commands.ClimberCommands.Reel;
import frc.robot.commands.ElevatorCommands.HomeElevator;
import frc.robot.commands.ElevatorCommands.RunElevator;
import frc.robot.commands.EndeffectorCommands.Slurp;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.Factories.IntakeFactory;
import frc.robot.commands.Factories.ScoringFactory;
import frc.robot.commands.SwerveCommands.AligntoCage;
import frc.robot.commands.SwerveCommands.AligntoReef;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
        // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                        .in(MetersPerSecond) * 0.1)
                        .withRotationalDeadband(
                                        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond
                                                        * 0.1) // Add a
                                                               // 10%
                                                               // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final Telemetry logger = new Telemetry(
                        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond));

        private final CommandXboxController xDrive = new CommandXboxController(
                        OperatorConstants.kDriverControllerPort);

        private final CommandXboxController xManip = new CommandXboxController(
                        OperatorConstants.kManipControllerPort);

        private final GenericHID xSim = new GenericHID(2);

        public final CommandSwerveDrivetrain drivetrain = Constants.SwerveConstants.TunerConstants.createDrivetrain();

        public RobotContainer() {
                if (Robot.isSimulation()) {
                        configureSimBindings();
                } else {
                        configureDriverBindings();
                        configureManipBindings();
                }

                if (Robot.isSimulation()) {
                        drivetrain.setDefaultCommand(
                                        drivetrain.applyRequest(() -> drive.withVelocityX((xSim.getRawAxis(0))
                                                        * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                                                        .in(MetersPerSecond))
                                                        .withVelocityY((-xSim.getRawAxis(1))
                                                                        * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                                                                        .in(MetersPerSecond))
                                                        .withRotationalRate((xSim.getRawAxis(2))
                                                                        *
                                                                        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond)));
                } else {
                        // drivetrain.setDefaultCommand(
                        // // Drivetrain will execute this command periodically
                        // drivetrain.applyRequest(() -> drive.withVelocityX((-xDrive.getLeftY())
                        // * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                        // .in(MetersPerSecond))
                        // .withVelocityY((-xDrive.getLeftX())
                        // * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                        // .in(MetersPerSecond))
                        // .withRotationalRate((-xDrive.getRightX())
                        // *
                        // Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond)));
                }

        }

        private void configureDriverBindings() {

                xDrive.y().whileTrue(new Spit());
                xDrive.a().toggleOnTrue(ScoringFactory.getCoral());
                xDrive.x().toggleOnTrue(ScoringFactory.L4Position());
                xDrive.b().toggleOnTrue(ScoringFactory.returnHome());
                xDrive.povDown().whileTrue(new HomeElevator());
                xDrive.povUp().toggleOnTrue(new Spit());

                new Trigger(() -> Math.abs(xDrive.getLeftY()) > 0.02)
                                .whileTrue(new RunElevator(() -> xDrive.getLeftY()));

                // xDrive.x().toggleOnTrue(new AligntoReef(() -> -xDrive.getLeftY()
                // * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                // .in(MetersPerSecond),
                // () -> -xDrive.getLeftX()
                // * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                // .in(MetersPerSecond),
                // false));

                // xDrive.y().toggleOnTrue(new AligntoReef(() -> -xDrive.getLeftY()
                // * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                // .in(MetersPerSecond),
                // () -> -xDrive.getLeftX()
                // * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                // .in(MetersPerSecond),
                // true));

                // xDrive.a().toggleOnTrue(new PathToPoint());
                // xDrive.b().toggleOnTrue(new PathToPoint());

                new Trigger(() -> Math.abs(xDrive.getLeftY()) > 0.02)
                                .whileTrue(new RunElevator(() -> xDrive.getLeftY()));

                // xDrive.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(
                // Robot.isRed() ?
                // FlippingUtil.flipFieldPose(Constants.ScoringConstants.kResetPose)
                // : Constants.ScoringConstants.kResetPose)));

                // reset the field-centric heading on left bumper press
                xDrive.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void configureManipBindings() {

                new Trigger(() -> Math.abs(xManip.getLeftY()) > 0.01)
                                .whileTrue(new RunArm(() -> xManip.getLeftY()));
                new Trigger(() -> Math.abs(xManip.getRightY()) > 0.01)
                                .whileTrue(new RunElevator(() -> xManip.getLeftY()));

                xManip.x().toggleOnTrue(ScoringFactory.L1Score());
                xManip.y().toggleOnTrue(ScoringFactory.L2Score());
                xManip.b().toggleOnTrue(ScoringFactory.L3Score());
                xManip.a().toggleOnTrue(ScoringFactory.L4Score());

                xManip.x().and(xManip.leftTrigger(0.1)).toggleOnTrue(ScoringFactory.L1Position());
                xManip.y().and(xManip.leftTrigger(0.1)).toggleOnTrue(ScoringFactory.L2Position());
                xManip.b().and(xManip.leftTrigger(0.1)).toggleOnTrue(ScoringFactory.L3Position());
                xManip.a().and(xManip.leftTrigger(0.1)).toggleOnTrue(ScoringFactory.L4Position());

                xManip.povDown().whileTrue(new Slurp());

                xManip.rightBumper().toggleOnTrue(new HomeElevator());
                xManip.leftBumper().whileTrue(new Spit());

                xManip.rightStick().toggleOnTrue(new Deploy());
                xManip.leftStick().toggleOnTrue(new Reel());

                xManip.rightTrigger(0.1).toggleOnTrue(IntakeFactory.SourceIntake());
        }

        private void configureSimBindings() {

                new Trigger(() -> xSim.getRawButtonPressed(1))
                                .toggleOnTrue(
                                                ScoringFactory.L4Position());

                new Trigger(() -> xSim.getRawButton(2)).toggleOnTrue(
                                ScoringFactory.returnHome());

                new Trigger(() -> xSim.getRawButton(3))
                                .toggleOnTrue(new AligntoCage(() -> xSim.getRawAxis(0)
                                                * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                                                .in(MetersPerSecond),
                                                () -> xSim.getRawAxis(1)
                                                                * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                                                                .in(MetersPerSecond),
                                                1));
                // new AligntoReef(() -> -xSim.getRawAxis(0),
                // () -> -xSim.getRawAxis(1),
                // 11, true)

                // new Trigger(() -> m_driverControllerSim.getRawButtonPressed(1))
                // .whileTrue(new Lift(10d));
                // new Trigger(() -> xSim.getRawButtonPressed(1))
                // .toggleOnTrue(
                // // new MoveArm(400d)
                // new Lift(5d)
                // // ScoringFactory.L1Score()
                // );

                // new Trigger(() -> m_driverControllerSim.getRawButtonPressed(2)).whileTrue(
                // new Lift(60d)
                // );

        }

}
