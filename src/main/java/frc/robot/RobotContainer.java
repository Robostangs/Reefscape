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
import frc.robot.commands.ClimberCommands.Deploy;
import frc.robot.commands.ClimberCommands.Reel;
import frc.robot.commands.ElevatorCommands.HomeElevator;
import frc.robot.commands.ElevatorCommands.RunElevator;
import frc.robot.commands.EndeffectorCommands.Slurp;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.Factories.IntakeFactory;
import frc.robot.commands.Factories.ScoringFactory;
import frc.robot.commands.Factories.ScoringFactory.ScoringPosition;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.Heimlich;
import frc.robot.commands.IntakeCommands.HomeIntake;
import frc.robot.commands.IntakeCommands.ManualIntake;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.IntakeCommands.Untake;
import frc.robot.commands.SwerveCommands.AligntoCage;
import frc.robot.commands.SwerveCommands.AligntoReef;
import frc.robot.commands.SwerveCommands.PathToPoint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakePivot;

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

                // xTest.a().whileTrue(new HomeElevator());

                xTest.a().onTrue(IntakePivot.getInstance().runOnce(IntakePivot.getInstance().zeroIntakeRun));


                xTest.x().toggleOnTrue(new Retract());
                xTest.y().toggleOnTrue(new Extend());
                //xTest.b().toggleOnTrue(new RunIntake());
                xTest.b().toggleOnTrue(new Heimlich());


                xTest.povUp().whileTrue(new HomeIntake());
                new Trigger(() -> Math.abs(xTest.getLeftY()) > 0.02)
                                .whileTrue(new ManualIntake(() -> xTest.getLeftY()*0.25));
                // new Trigger(() -> Math.abs(xTest.getLeftY()) > 0.01)
                // .whileTrue(new RunArm(() -> xTest.getLeftY()));

                new Trigger(() -> Math.abs(xTest.getRightY()) > 0.02)
                                .whileTrue(new RunElevator(() -> -xTest.getRightY()));

        }

        private void configureDriverBindings() {

                new Trigger(() -> DriverStation.getMatchTime() > 30).and(() -> DriverStation.getMatchTime() < 15)
                                .onTrue(
                                                new RunCommand(() -> xDrive.setRumble(RumbleType.kBothRumble, 0.5)))
                                .onFalse(
                                                new RunCommand(() -> xDrive.setRumble(RumbleType.kBothRumble, 0)));

                xDrive.rightStick().toggleOnTrue(IntakeFactory.IntakeCoral().finallyDo( Retract.Retract));

                xDrive.b().toggleOnTrue(IntakeFactory.Vomit());

                xDrive.y().toggleOnTrue(new Untake());
                xDrive.x().toggleOnTrue(new Retract());

                xDrive.a().toggleOnTrue(new AligntoCage(() -> xDrive.getLeftX(), () -> xDrive.getLeftY(), 2));

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

                new Trigger(() -> DriverStation.getMatchTime() > 30).and(() -> DriverStation.getMatchTime() < 15)
                                .onTrue(
                                                new RunCommand(() -> xManip.setRumble(RumbleType.kBothRumble, 0.5)))
                                .onFalse(
                                                new RunCommand(() -> xManip.setRumble(RumbleType.kBothRumble, 0)));

                new Trigger(() -> Math.abs(xManip.getLeftY()) > 0.1)
                                .whileTrue(new RunArm(() -> xManip.getLeftY() / 2));
                new Trigger(() -> Math.abs(xManip.getRightY()) > 0.1)
                                .whileTrue(new RunElevator(() -> -xManip.getRightY() / 2));

                xManip.a().toggleOnTrue(ScoringFactory.L4Position().finallyDo(ScoringFactory.returnStow()));
                xManip.b().toggleOnTrue(ScoringFactory.L3Position().finallyDo(ScoringFactory.returnStow()));
                xManip.y().toggleOnTrue(ScoringFactory.L2Position().finallyDo(ScoringFactory.returnStow()));
                xManip.x().toggleOnTrue(ScoringFactory.SourceIntake());

                xManip.povDown().toggleOnTrue(new Slurp());
                xManip.povRight().toggleOnTrue(ScoringFactory.SchloopCommand());
                xManip.povLeft().toggleOnTrue(ScoringFactory.Stow());


                xManip.b().and(xManip.rightTrigger(0.2)).toggleOnTrue(ScoringFactory.ByeByeByeAlge(ScoringPosition.L3));


                xManip.rightStick().toggleOnTrue(new Deploy(true));
                xManip.leftStick().toggleOnTrue(new Reel(true));


                // xManip.povUp().onTrue(new Spit().withTimeout(0.1).andThen(new Slurp().withTimeout(0.1).onlyIf(
                //         xManip.rightTrigger(0.3)
                // )));

                xManip.povUp().onTrue(Climber.getInstance().runOnce(Climber.getInstance().zeroClimberPosition));

                xManip.rightBumper().toggleOnTrue(new HomeElevator());
                xManip.leftBumper().whileTrue(new Spit());

        }

        private void configureSimBindings() {

                new Trigger(() -> xSim.getRawButtonPressed(1)).onTrue(drivetrain.runOnce(() -> drivetrain.resetPose(
                                Robot.isRed() ? FlippingUtil.flipFieldPose(Constants.ScoringConstants.kResetPose)
                                                : Constants.ScoringConstants.kResetPose)));

                new Trigger(() -> xSim.getRawButton(2)).onTrue(
                                new AligntoReef(true)
                                );
                                

                new Trigger(() -> xSim.getRawButton(3))
                                .onTrue(new PathToPoint(Constants.ScoringConstants.kResetPose));

        }

}
