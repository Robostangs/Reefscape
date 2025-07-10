// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.FlippingUtil;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommands.SetArmPosition;
import frc.robot.commands.ArmCommands.SetArmDutyCycle;
import frc.robot.commands.ClimberCommands.Deploy;
import frc.robot.commands.ClimberCommands.ManualAdjustClimber;
import frc.robot.commands.ClimberCommands.Reel;
import frc.robot.commands.ElevatorCommands.HomeElevator;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.ElevatorCommands.SetElevatorDutyCycle;
import frc.robot.commands.EndeffectorCommands.Slurp;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.Factories.IntakeFactory;
import frc.robot.commands.Factories.ScoringFactory;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.HomeIntake;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.commands.IntakeCommands.Untake;
import frc.robot.commands.SwerveCommands.AligntoReef;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;

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

        public static boolean useVision = true;

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
                xTest.leftStick().whileTrue(new Slurp(false));

                xTest.a().toggleOnTrue(new HomeElevator());

                xTest.x().toggleOnTrue(new Retract());
                xTest.y().toggleOnTrue(new Extend(false));
                xTest.b().toggleOnTrue(new RunIntake());

                xTest.povLeft().whileTrue(new HomeIntake());

                xTest.povUp().toggleOnTrue(new SetElevatorPosition(Constants.ScoringConstants.L3.kElevatorPos));
                xTest.povRight().toggleOnTrue(new SetArmPosition(Constants.ScoringConstants.Stow.kArmStowPos));

                xTest.povDown().toggleOnTrue(ScoringFactory.SmartStow());

                new Trigger(() -> Math.abs(xTest.getLeftY()) > 0.01)
                                .whileTrue(new SetArmDutyCycle(() -> xTest.getLeftY()));

                new Trigger(() -> Math.abs(xTest.getRightY()) > 0.02)
                                .whileTrue(new SetElevatorDutyCycle(() -> -xTest.getRightY()));

        }

        private void configureDriverBindings() {

                new Trigger(() -> Timer.getMatchTime() < 25).and(() -> Timer.getMatchTime() > 20)
                                .onTrue(
                                                new RunCommand(() -> xDrive.getHID().setRumble(RumbleType.kBothRumble,
                                                                1)))
                                .onFalse(
                                                new RunCommand(() -> xDrive.setRumble(RumbleType.kBothRumble, 0)));

                xDrive.rightStick().toggleOnTrue(IntakeFactory.IntakeCoral());
                xDrive.leftStick().toggleOnTrue(new HomeIntake());

                xDrive.leftBumper().toggleOnTrue(AligntoReef.getAlignToReef(() -> false));
                xDrive.rightBumper().toggleOnTrue(AligntoReef.getAlignToReef(() -> true));

                xDrive.rightTrigger().toggleOnTrue(IntakeFactory.algaeOut());

                xDrive.b().toggleOnTrue(new RunIntake());

                //TAGS:
                /**
                 *red: 6,7,8,9,10,11
                 *blue: 17,18,19,20,21,22
                 */

                xDrive.y().toggleOnTrue(new Untake());
                xDrive.x().toggleOnTrue(new Retract());
                xDrive.a().toggleOnTrue(IntakeFactory.algaeIn());

                xDrive.povLeft().toggleOnTrue(Climber.getInstance().runOnce(Climber.getInstance().zeroClimberPosition));

                xDrive.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(
                                Robot.isRed() ? FlippingUtil.flipFieldPose(Constants.ScoringConstants.kResetPose)
                                                : Constants.ScoringConstants.kResetPose)));

                xDrive.povRight().onTrue(new InstantCommand(
                                (() -> useVision = !useVision)));
                xDrive.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                            
                // reset the field-centric hea ding on left bumper press

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void configureManipBindings() {

                new Trigger(() -> Timer.getMatchTime() < 25).and(() -> Timer.getMatchTime() > 20)
                                .onTrue(
                                                new RunCommand(() -> xManip.getHID().setRumble(RumbleType.kBothRumble,
                                                                1)))
                                .onFalse(
                                                new RunCommand(() -> xManip.getHID().setRumble(RumbleType.kBothRumble,
                                                                0)));

                // new Trigger(() -> Math.abs(xManip.getLeftY()) > 0.1)
                //                 .whileTrue(new SetArmDutyCycle(() -> xManip.getLeftY() / 2));

                
                new Trigger(() -> Math.abs(xManip.getLeftY()) > 0.1)
                                .whileTrue(new ManualAdjustClimber(() -> xManip.getLeftY() / 2));

                new Trigger(() -> Math.abs(xManip.getRightY()) > 0.1)
                                .whileTrue(new SetElevatorDutyCycle(() -> -xManip.getRightY() / 2));

                xManip.a().toggleOnTrue(
                                // ScoringFactory.L4Pos0itionAuto()
                                ScoringFactory.L4Score(xManip.leftBumper()).andThen(ScoringFactory.SmartStow())
                                );
                xManip.b().toggleOnTrue(
                                ScoringFactory.L3Score(xManip.leftBumper()).andThen(ScoringFactory.SmartStow()));
                xManip.y().toggleOnTrue(
                                ScoringFactory.L2Score(xManip.leftBumper()).andThen(ScoringFactory.SmartStow()));
                xManip.x().toggleOnTrue(
                                ScoringFactory.L1Score(xManip.leftBumper()).andThen(ScoringFactory.SmartStow()));

                xManip.b().and(xManip.rightTrigger(0.2)).toggleOnTrue(ScoringFactory.ByeByeByeAlgaeL3());
                xManip.y().and(xManip.rightTrigger(0.2)).toggleOnTrue(ScoringFactory
                                .ByeByeByeAlgaeL2());

                xManip.povDown().whileTrue(new Slurp(false));
                xManip.povRight().toggleOnTrue(ScoringFactory.Schloop());
                xManip.povLeft().toggleOnTrue(ScoringFactory.SmartStow());
                xManip.povUp().whileTrue(new Spit());

                xManip.rightStick().toggleOnTrue(new Deploy(true));
                xManip.leftStick().toggleOnTrue(new Reel(true));

                // xManip.povUp().onTrue(Elevator.getInstance().runOnce(Elevator.getInstance().setHomePositionElevator));

                xManip.rightBumper().toggleOnTrue(
                                new HomeElevator().andThen(ScoringFactory.SmartStow()));
        }

        private void configureSimBindings() {

                new Trigger(() -> xSim.getRawButtonPressed(1)).onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(
                                Robot.isRed() ? FlippingUtil.flipFieldPose(Constants.ScoringConstants.kResetPose)
                                                : Constants.ScoringConstants.kResetPose)));

                new Trigger(() -> xSim.getRawButton(2)).onTrue(
                                AligntoReef.getAlignToReef(() -> true));

                new Trigger(() -> xSim.getRawButton(3))
                                .onTrue(AligntoReef.getAlignToReef(() -> false));

        }

}
