// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.FlippingUtil;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveArm;
import frc.robot.commands.RunArm;
import frc.robot.commands.ElevatorCommands.HomeElevator;
import frc.robot.commands.ElevatorCommands.RunElevator;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.IntakeCommands.RunIntake;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
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
                configureDriverBindings();
                if (Robot.isSimulation()) {
                        configureSimBindings();
                }
        }

        private void configureDriverBindings() {
                // if (Robot.isSimulation()) {
                //         drivetrain.setDefaultCommand(
                //                         drivetrain.applyRequest(() -> drive.withVelocityX((xSim.getRawAxis(0))
                //                                         * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                //                                                         .in(MetersPerSecond))
                //                                         .withVelocityY((-xSim.getRawAxis(1))
                //                                                         * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                //                                                                         .in(MetersPerSecond))
                //                                         .withRotationalRate((xSim.getRawAxis(2))
                //                                                         *
                //                                                         Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond)));
                // } else {
                //         drivetrain.setDefaultCommand(
                //                         // Drivetrain will execute this command periodically
                //                         drivetrain.applyRequest(() -> drive.withVelocityX((-xDrive.getLeftY())
                //                                         * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                //                                                         .in(MetersPerSecond))
                //                                         .withVelocityY((-xDrive.getLeftX())
                //                                                         * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                //                                                                         .in(MetersPerSecond))
                //                                         .withRotationalRate((-xDrive.getRightX())
                //                                                         *
                //                                                         Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond)));
                // }

                // new Trigger(() -> (xDrive.getLeftY() >= 0.1))
                // .whileTrue(new RunElevator(() -> xDrive.getLeftY() * 0.075));

                // new Trigger(() -> (xDrive.getRightY() >= 0.1))
                // .whileTrue(new RunElevator(() -> xDrive.getRightY() * -0.075));

                // xDrive.a().toggleOnTrue(new RunArm(() -> 0.1));
                // xDrive.b().toggleOnTrue(new RunArm(() -> -0.1));

                // xDrive.x().whileTrue(new Spit());

                // xDrive.y().toggleOnTrue(new Retract());
                // xDrive.x().toggleOnTrue(new Extend());

                // xDrive.rightStick().toggleOnTrue(new Extend().andThen(new RunIntake()).finallyDo(Retract.Retract));

               // new Trigger(() -> Math.abs(xDrive.getLeftY()) > 0.05).whileTrue(new RunArm(() -> xDrive.getLeftY(), false));
                xDrive.rightStick().toggleOnTrue(new RunIntake());
                xDrive.leftStick().toggleOnTrue(new Extend());
                xDrive.povRight().toggleOnTrue(new Retract());
                xDrive.povLeft().toggleOnTrue(new HomeElevator());


                xDrive.x().onTrue(Arm.getInstance().run(Arm.getInstance().gotoZero));
                xDrive.y().onTrue(Arm.getInstance().run(Arm.getInstance().gotoSchloop));

                

                // xDrive.x().toggleOnTrue(new RunElevator(() -> 0.15, true));
                // xDrive.y().toggleOnTrue(new RunElevator(() -> -0.03, false));
                xDrive.a().toggleOnTrue(new RunArm(() ->0.1 ,true));
                xDrive.b().toggleOnTrue(new RunArm(() ->-0.1 ,true));
                xDrive.leftBumper().whileTrue(new Spit());


                


                // xDrive.b().toggleOnTrue(new HomeElevator());

                xDrive.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(
                                Robot.isRed() ? FlippingUtil.flipFieldPose(Constants.ScoringConstants.kResetPose)
                                                : Constants.ScoringConstants.kResetPose)));

                // reset the field-centric heading on left bumper press
                xDrive.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void configureManipBindings() {

        }

        private void configureSimBindings() {

                new Trigger(() -> xSim.getRawButtonPressed(1))
                                .toggleOnTrue(
                                                // new Lift(20d)

                                                new MoveArm(400d));
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

        public Command getAutonomousCommand() {
                return Commands.print("No autonomous command configured");
        }
}
