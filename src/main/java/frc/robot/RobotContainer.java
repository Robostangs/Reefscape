// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.FlippingUtil;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommands.RunArm;
import frc.robot.commands.ElevatorCommands.HomeElevator;
import frc.robot.commands.ElevatorCommands.RunElevator;
import frc.robot.commands.EndeffectorCommands.Slurp;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.Factories.IntakeFactory;
import frc.robot.commands.Factories.ScoringFactory;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.HomeIntake;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.Untake;
import frc.robot.commands.SwerveCommands.AligntoCage;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
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

        private final CommandXboxController xTest = new CommandXboxController(
                        2);

        private final GenericHID xSim = new GenericHID(2);

        public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

        public RobotContainer() {
                if (Robot.isSimulation()) {
                        configureSimBindings();
                } else {
                        configureDriverBindings();
                        configureManipBindings();
                        configureTestBindings();
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
                        drivetrain.setDefaultCommand(
                                        // Drivetrain will execute this command periodically
                                        drivetrain.applyRequest(() -> drive.withVelocityX((-xDrive.getLeftY())
                                                        * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                                                        .in(MetersPerSecond)
                                                        * ((xDrive.getLeftTriggerAxis() > 0.2) ? 0.25 : 1))
                                                        .withVelocityY((-xDrive.getLeftX())
                                                                        * ((xDrive.getLeftTriggerAxis() > 0.2) ? 0.25
                                                                                        : 1)
                                                                        * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                                                                        .in(MetersPerSecond))
                                                        .withRotationalRate((-xDrive.getRightX())
                                                                        *
                                                                        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond
                                                                        * ((xDrive.getLeftTriggerAxis() > 0.2) ? 0.25
                                                                                        : 1))
                                                        .withRotationalDeadband(
                                                                        Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond
                                                                                        * 0.05
                                                                                        * ((xDrive.getLeftTriggerAxis() > 0.2)
                                                                                                        ? 0.25
                                                                                                        : 1))
                                                        .withDeadband(Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                                                        .in(MetersPerSecond) * 0.05
                                                                        * ((xDrive.getLeftTriggerAxis() > 0.2)
                                                                                        ? 0.25
                                                                                        : 1))));
                }

        }

        private void configureTestBindings() {

                xTest.rightStick().whileTrue(new Spit());
                xTest.leftStick().whileTrue(new Slurp());

                xTest.a().whileTrue(new HomeElevator());

                xTest.x().toggleOnTrue(new Retract());
                xTest.y().toggleOnTrue(new Extend());
                xTest.b().toggleOnTrue(new RunIntake());

                // xTest.a().whileTrue(Arm.getInstance().run(Arm.getInstance().gotoSchloop));

                xTest.povUp().whileTrue(new HomeIntake());
                new Trigger(() -> Math.abs(xTest.getLeftY()) > 0.01)
                                .whileTrue(new RunArm(() -> xTest.getLeftY()));

                new Trigger(() -> Math.abs(xTest.getRightY()) > 0.02)
                                .whileTrue(new RunElevator(() -> -xTest.getRightY()));

        }

        private void configureDriverBindings() {

                xDrive.rightStick().toggleOnTrue(IntakeFactory.IntakeCoral());

                xDrive.b().toggleOnTrue(IntakeFactory.Vomit());

                xDrive.y().toggleOnTrue(new Untake());
                xDrive.x().toggleOnTrue(new Retract());

                xDrive.leftStick().toggleOnTrue(new HomeIntake());

                xDrive.povLeft().toggleOnTrue(Climber.getInstance().runOnce(Climber.getInstance().zeroClimberPosition));


                xDrive.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(
                                Robot.isRed() ? FlippingUtil.flipFieldPose(Constants.ScoringConstants.kResetPose)
                                                : Constants.ScoringConstants.kResetPose)));

                                                
                // reset the field-centric heading on left bumper press
                xDrive.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void configureManipBindings() {

                new Trigger(() -> Math.abs(xManip.getLeftY()) > 0.1)
                                .whileTrue(new RunArm(() -> xManip.getLeftY()/2));
                new Trigger(() -> Math.abs(xManip.getRightY()) > 0.1)
                                .whileTrue(new RunElevator(() -> -xManip.getRightY()/2));

                xManip.a().toggleOnTrue(ScoringFactory.L4Position().finallyDo(ScoringFactory.returnStow()));
                xManip.b().toggleOnTrue(ScoringFactory.L3Position().finallyDo(ScoringFactory.returnStow()));
                
                xManip.y().toggleOnTrue(ScoringFactory.L2Position().finallyDo(ScoringFactory.returnStowL2()));
                


                xManip.povDown().toggleOnTrue(new Slurp());

                xManip.povRight().toggleOnTrue(ScoringFactory.SchloopCommand());
                xManip.povUp().toggleOnTrue(ScoringFactory.StowL2());
                xManip.povLeft().toggleOnTrue(ScoringFactory.Stow());

                xManip.rightBumper().toggleOnTrue(new HomeElevator());
                xManip.leftBumper().whileTrue(new Spit());

                // xManip.rightStick().and(xManip.leftTrigger(0.1)).whileTrue(new Deploy(true));
                // xManip.leftStick().and(xManip.leftTrigger(0.1)).whileTrue(new Reel(true));

                // xManip.rightStick().and(xManip.rightTrigger(0.1)).whileTrue(new Deploy(false));
                // xManip.leftStick().and(xManip.rightTrigger(0.1)).whileTrue(new Reel(false));

        }

        private void configureSimBindings() {

                // new Trigger(() -> xSim.getRawButtonPressed(1))
                // .toggleOnTrue(
                // ScoringFactory.L4Position());

                new Trigger(() -> xSim.getRawButtonPressed(1)).onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(
                                Robot.isRed() ? FlippingUtil.flipFieldPose(Constants.ScoringConstants.kResetPose)
                                                : Constants.ScoringConstants.kResetPose)));

                new Trigger(() -> xSim.getRawButton(2)).toggleOnTrue(
                                ScoringFactory.Stow());

                new Trigger(() -> xSim.getRawButton(3))
                                .toggleOnTrue(new AligntoCage(() -> xSim.getRawAxis(0)
                                                * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                                                .in(MetersPerSecond),
                                                () -> xSim.getRawAxis(1)
                                                                * Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts
                                                                                .in(MetersPerSecond),
                                                1));

        }

}
