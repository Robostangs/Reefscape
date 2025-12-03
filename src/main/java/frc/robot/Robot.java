// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ArmCommands.SetArmDutyCycle;
import frc.robot.commands.ArmCommands.SetArmPosition;
import frc.robot.commands.ClimberCommands.Deploy;
import frc.robot.commands.ClimberCommands.Reel;
import frc.robot.commands.ElevatorCommands.HomeElevator;
import frc.robot.commands.ElevatorCommands.SetElevatorDutyCycle;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.EndeffectorCommands.Slurp;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.Factories.ScoringFactory;
import frc.robot.commands.IntakeCommands.Extend;
import frc.robot.commands.IntakeCommands.HomeIntake;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.IntakeCommands.RunIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Endeffector;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;

public class Robot extends TimedRobotstangs {

    private final RobotContainer m_robotContainer;

    private CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

    public static Field2d teleopField = new Field2d();

    //Shuffleboard tabs
    public static ShuffleboardTab autoTab, teleopTab, testTab, disTab;

    //Alerts
    private static Alert gcAlert = new Alert("MEMORY TWEAKING FIX RN", AlertType.kError);
    private static Alert CANcoderAlert = new Alert("Can tweaking", AlertType.kError);
    private static Alert MotorAlert = new Alert("Can tweaking", AlertType.kError);

    public boolean testConfigured = false;

    //Pit Test Choosers
    public static SendableChooser<Command> SwerveCommands = new SendableChooser<>();
    public static SendableChooser<Command> ElevatorCommands = new SendableChooser<>();
    public static SendableChooser<Command> IntakeCommands = new SendableChooser<>();
    public static SendableChooser<Command> EndeffectorCommands = new SendableChooser<>();
    public static SendableChooser<Command> ArmCommands = new SendableChooser<>();
    public static SendableChooser<Command> ClimberCommands = new SendableChooser<>();
    public static SendableChooser<Command> AlgaeffectorCommands = new SendableChooser<>();
    Command lastSwerve,lastClimber,lastIntake,lastElevator, lastEndaffector, lastArm, lastAlgaeffector;
    SequentialCommandGroup autoGroup;
    private static String autoName = "";

    // Autos
    private SendableChooser<String> startChooser = new SendableChooser<>();
    private SendableChooser<String> firstPieceChooser = new SendableChooser<>();
    private SendableChooser<String> firstPieceRoLChooser = new SendableChooser<>();
    private SendableChooser<String> secondPieceChooser = new SendableChooser<>();
    private SendableChooser<String> secondPieceRoLChooser = new SendableChooser<>();
    private SendableChooser<String> thirdPieceChooser = new SendableChooser<>();
    private SendableChooser<String> thirdPieceRoLChooser = new SendableChooser<>();

    static NetworkTableEntry pathDelay;

    private Command autoCommand;

    private static GcStatsCollector gscollect = new GcStatsCollector();
    private static String lastAutoName;
    
    //Alerts
    private static Alert nullAuto = new Alert("Null auto", AlertType.kWarning);
    private static Alert publishfail = new Alert("Publishing failed", AlertType.kError);
    private static Alert noAutoSelected = new Alert("No Auto Selected", AlertType.kWarning);
    private static Alert PissingAlert = new Alert("We going forward ", AlertType.kInfo);
    private Timer timer = new Timer();

    private String oldAutoName = "";

  public Robot() {
    m_robotContainer = new RobotContainer();

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void robotInit() {

    // Making the dashboard on Elastic
    SmartDashboard.putData("Field", teleopField);
    teleopTab = Shuffleboard.getTab("Teleoperated");
    autoTab = Shuffleboard.getTab("Autonomous");
    testTab = Shuffleboard.getTab("Test");
    disTab = Shuffleboard.getTab("Disabled");

    //Auto Choosers
    startChooser.setDefaultOption("Shit and Shit", "");
    startChooser.addOption("Forward", "Pissing");
    startChooser.addOption("Dumb L4", "Shitting");

    startChooser.addOption("PTP to Center 2R ", "PTP");
    startChooser.addOption("Center", "CStart");
    startChooser.addOption("Open", "OStart");
    startChooser.addOption("Processor", "PStart");

    firstPieceChooser.setDefaultOption("none", "");
    firstPieceChooser.addOption("Center 1", " - C1");
    firstPieceChooser.addOption("Center 2", " - C2");
    firstPieceChooser.addOption("Pro 1", " - P1");
    firstPieceChooser.addOption("Pro 2", " - P2");
    firstPieceChooser.addOption("Open 1", " - O1");
    firstPieceChooser.addOption("Open 2", " - O2");

    firstPieceRoLChooser.setDefaultOption("None", "");
    firstPieceRoLChooser.setDefaultOption("Right", "R");
    firstPieceRoLChooser.addOption("Left", "L");

    secondPieceChooser.setDefaultOption("None", "");
    secondPieceChooser.addOption("Center 1", " - C1");
    secondPieceChooser.addOption("Center 2", " - C2");
    secondPieceChooser.addOption("Pro 1", " - P1");
    secondPieceChooser.addOption("Pro 2", " - P2");
    secondPieceChooser.addOption("Open 1", " - O1");
    secondPieceChooser.addOption("Open 2", " - O2");

    secondPieceRoLChooser.setDefaultOption("None", "");
    secondPieceRoLChooser.addOption("Right", "R");
    secondPieceRoLChooser.addOption("Left", "L");

    thirdPieceChooser.setDefaultOption("None", "");
    thirdPieceChooser.addOption("Center 1", " - C1");
    thirdPieceChooser.addOption("Center 2", " - C2");
    thirdPieceChooser.addOption("Pro 1", " - P1");
    thirdPieceChooser.addOption("Pro 2", " - P2");
    thirdPieceChooser.addOption("Open 1", " - O1");
    thirdPieceChooser.addOption("Open 2", " - O2");

    thirdPieceRoLChooser.setDefaultOption("None", "");
    thirdPieceRoLChooser.addOption("Right", "R");
    thirdPieceRoLChooser.addOption("Left", "L");
    thirdPieceRoLChooser.addOption("Open 4 piece", "OStart - O2L - O1R - O1L - C1L");

    //Putting the Choosers in the Shuffleboard Tab
    autoTab.add("Start Chooser", startChooser)
        .withSize(2, 1)
        .withPosition(0, 0);

    autoTab.add("First Piece Chooser", firstPieceChooser)
        .withSize(2, 1)
        .withPosition(0, 1);

    autoTab.add("First Piece Right or Left", firstPieceRoLChooser)
        .withSize(1, 1)
        .withPosition(2, 1);

    autoTab.add("Second Piece Chooser", secondPieceChooser)
        .withSize(2, 1)
        .withPosition(0, 2);

    autoTab.add("Second Piece Right or Left", secondPieceRoLChooser)
        .withSize(1, 1)
        .withPosition(2, 2);

    autoTab.add("Third Piece Chooser", thirdPieceChooser)
        .withSize(2, 1)
        .withPosition(0, 3);

    autoTab.add("Third Piece Right or Left", thirdPieceRoLChooser)
        .withSize(1, 1)
        .withPosition(2, 3);

    autoTab.add("Path Delay", 0)
        .withSize(3, 1)
        .withPosition(4, 4)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min_value", 0, "max_value", 15, "block increment", 15, "Divisions", 6));

    pathDelay = NetworkTableInstance.getDefault().getTable("Shuffleboard")
        .getSubTable(autoTab.getTitle())
        .getEntry("Path Delay");

    autoName = startChooser.getSelected() + firstPieceChooser.getSelected() + firstPieceRoLChooser.getSelected()
        + secondPieceChooser.getSelected() + secondPieceRoLChooser.getSelected()
        + thirdPieceChooser.getSelected() + thirdPieceRoLChooser.getSelected();

    teleopTab.addNumber("Match Time", () -> Timer.getMatchTime())
        .withSize(3, 4)
        .withPosition(3, 0)
        .withWidget("Match Time")
        .withProperties(Map.of("red_start_time", 15, "yellow_start_time", 30));

    // Adding Path Planner Commands
    NamedCommands.registerCommand("L3 Score", ScoringFactory.L3ScoreAuto().andThen(ScoringFactory.SmartStow()));
    NamedCommands.registerCommand("L4 Score", ScoringFactory.L4ScoreAuto().andThen(ScoringFactory.SmartStow()));

    NamedCommands.registerCommand("L4 Position", ScoringFactory.L4PositionAuto());
    NamedCommands.registerCommand("L4 Position regular", ScoringFactory.L4Position());

    NamedCommands.registerCommand("L3 Position", ScoringFactory.L3PositionAuto());

    NamedCommands.registerCommand("Spit", new Spit().withTimeout(0.4));

    NamedCommands.registerCommand("Ground Intake", new Extend(true));
    NamedCommands.registerCommand("Retract", new Retract(true).withTimeout(0.5));
    NamedCommands.registerCommand("Intake", new PrintCommand("Hopefully autos work"));

    NamedCommands.registerCommand("Stow", ScoringFactory.SmartStow());
    NamedCommands.registerCommand("Schloop", ScoringFactory.Schloop().withTimeout(0.4));



  }



    @Override
    public void testInit() {
        // Close commands
        SwerveCommands.close();
        ArmCommands.close();
        ElevatorCommands.close();
        ClimberCommands.close();
        IntakeCommands.close();
        EndeffectorCommands.close();
        //pit test
        if (!testConfigured) {
            // Swerve Drivetrain
            //When pressed, the swerve drivetrain will reset the gyro and set the pose to the center of the field
            SwerveCommands.setDefaultOption("Do nothin(basically reseting the gyro)",
                    CommandSwerveDrivetrain.getInstance()
                            .runOnce(() -> CommandSwerveDrivetrain.getInstance()
                            .resetPose(!Robot.isRed()
                                    ? Constants.SwerveConstants.AutoConstants.AutoPoses.kCenterPose
                                    : FlippingUtil
                                            .flipFieldPose(Constants.SwerveConstants.AutoConstants.AutoPoses.kCenterPose))));

            // An option on elastic when pressed will make the swerve drivetrain's wheels move forward
            SwerveCommands.addOption("Drive forward",
                    CommandSwerveDrivetrain.getInstance().applyRequest(() -> new SwerveRequest.FieldCentric()
                    .withVelocityX(
                            Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12VoltsPIT.in(MetersPerSecond)))
                            .withName("Drive Forward"));

            // An option on elastic when pressed will make the swerve drivetrain's wheels move backward
            SwerveCommands.addOption("Drive Backward",
                    CommandSwerveDrivetrain.getInstance().applyRequest(() -> new SwerveRequest.FieldCentric()
                    .withVelocityX(
                            -Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12VoltsPIT.in(MetersPerSecond)))
                            .withName("Drive Backward"));

            // An option on elastic when pressed will make the swerve drivetrain's wheels move left
            SwerveCommands.addOption("Drive Left",
                    CommandSwerveDrivetrain.getInstance().applyRequest(() -> new SwerveRequest.FieldCentric()
                    .withVelocityY(
                            Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12VoltsPIT.in(MetersPerSecond)))
                            .withName("Drive Left"));

            // An option on elastic when pressed will make the swerve drivetrain's wheels move right
            SwerveCommands.addOption("Drive Right",
                    CommandSwerveDrivetrain.getInstance().applyRequest(() -> new SwerveRequest.FieldCentric()
                    .withVelocityY(
                            Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12VoltsPIT.in(MetersPerSecond)))
                            .withName("Drive Right"));

            // An option on elastic when pressed will make the swerve drivetrain's wheels rotate
            SwerveCommands.addOption("Rotate",
                    CommandSwerveDrivetrain.getInstance().applyRequest(() -> new SwerveRequest.RobotCentric()
                    .withRotationalRate(
                            Constants.SwerveConstants.AutoConstants.AutoSpeeds.kMaxAngularSpeedRadiansPerSecond))
                            .withName("Rotate"));

            // The actual button on eleastic with its customizations
            testTab.add("Swerve", SwerveCommands)
                    .withSize(2, 1)
                    .withPosition(0, 0);

            // Elevator
            ElevatorCommands.setDefaultOption("nothin",
                    Elevator.getInstance()
                            .runOnce(() -> Elevator.getInstance().setElevatorDutyCycle(0)));

            // An option on elastic when pressed will make the elevator up in the given duty cycle
            ElevatorCommands.addOption("Elevator up",
                    new SetElevatorDutyCycle(() -> Constants.ElevatorConstants.ktestDutyCycle));

            // An option on elastic when pressed will make the elevator down in the given duty cycle
            ElevatorCommands.addOption("Elevator down",
                    new SetElevatorDutyCycle(() -> -Constants.ElevatorConstants.ktestDutyCycle));

            // An option on elastic when pressed will set the elevator to the L3 position
            ElevatorCommands.addOption("L3", new SetElevatorPosition(Constants.ScoringConstants.L3.kElevatorPos));

            // An option on elastic when pressed will set the elevator to the L2 position
            ElevatorCommands.addOption("L2", new SetElevatorPosition(Constants.ScoringConstants.L2.kElevatorEnd));

            // An option on elastic when pressed will set the elevator to the L4 position
            ElevatorCommands.addOption("L4", new SetElevatorPosition(Constants.ScoringConstants.L4.kElevatorPos));

            // An option on elastic when pressed will set the elevator to the stow position
            ElevatorCommands.addOption("Stow", new SetElevatorPosition(Constants.ScoringConstants.Stow.kElevatorPos));

            // An option on elastic when pressed will set the elevator to home the elevator
            ElevatorCommands.addOption("Home Elevator", new HomeElevator());

            // The actual button on elastic with its customizations
            testTab.add("Elevator", ElevatorCommands)
                    .withSize(2, 1)
                    .withPosition(0, 1);

            //sets the default option to nothin, which will set the pivot duty cycle to 0
            IntakeCommands.setDefaultOption("nothin",
                    IntakePivot.getInstance()
                            .runOnce(() -> IntakePivot.getInstance().setPiviotDutyCycle(0)));

            // An option on elastic when pressed will set the intake pivot to extend
            IntakeCommands.addOption("Extend", new Extend(false));

            //  An option on elastic when pressed will set the intake pivot to retract
            IntakeCommands.addOption("Retract", new Retract(true));

            // An option on elastic when pressed will set the intake pivot to just run the motors
            IntakeCommands.addOption("RunIntake", new RunIntake());

            // An option on elastic when pressed will set the intake pivot to home the intake
            IntakeCommands.addOption("Home Intake", new HomeIntake().withTimeout(3));

            // The actual button on elastic with its customizations
            testTab.add("IntakeCommands", IntakeCommands)    
                    .withSize(2, 1)
                    .withPosition(0, 2);

            // endeffector
            // sets the default option to nothin, which will set the endeffector duty cycle to 0
            EndeffectorCommands.setDefaultOption("nothin",
                    Endeffector.getInstance()
                            .runOnce(() -> Endeffector.getInstance().setEneffdector(0)));

            // An option on elastic when pressed will set the endeffector to "spit" the coral out
            EndeffectorCommands.addOption("Spit", new Spit());

            // An option on elastic when pressed will set the endeffector to "slurp" the coral in
            EndeffectorCommands.addOption("Slurp", new Slurp(false));

            // The actual button on elastic with its customizations
            testTab.add("EndeffectorCommands", EndeffectorCommands)
                    .withSize(2, 1)
                    .withPosition(0, 3);

            // Arm
            // sets the default option to nothin, which will set the arm duty cycle to 0
            ArmCommands.setDefaultOption("Nothin",
                    Arm.getInstance()
                            .runOnce(() -> Arm.getInstance().setArmDutyCycle(0)));

            // An option on elastic when pressed will set the arm to go up
            ArmCommands.addOption("postive", new SetArmDutyCycle(() -> Constants.ArmConstants.kArmDutyCycle));

            //  An option on elastic when pressed will set the arm to go down
            ArmCommands.addOption("negative", new SetArmDutyCycle(() -> -Constants.ArmConstants.kArmDutyCycle));

            // An option on elastic when pressed will set the arm to the L4 position
            ArmCommands.addOption("L4", new SetArmPosition(Constants.ScoringConstants.L4.kArmScoringPosition));

            // An option on elastic when pressed will set the arm to the L3 position
            ArmCommands.addOption("L3", new SetArmPosition(Constants.ScoringConstants.L3.kArmScoringPosition));

            // An option on elastic when pressed will set the arm to the L2 position
            ArmCommands.addOption("L2", new SetArmPosition(Constants.ScoringConstants.L2.kArmScoringPosition));

            // An option on elastic when pressed will set the arm to the stow position
            ArmCommands.addOption("Stow", new SetArmPosition(Constants.ScoringConstants.Stow.kArmStowPos));

            // The actual button on elastic with its customizations
            testTab.add("ArmCommands", ArmCommands)
                    .withSize(2, 1)
                    .withPosition(0, 4);
            // climber
            //sets the default option to nothin, which will set the climber duty cycle to 0
            ClimberCommands.setDefaultOption("Nothin",
                    Climber.getInstance()
                            .runOnce(() -> Climber.getInstance().runClimber(0)));

            // An option on elastic when pressed will deploy the climber(is a boolean)
            ClimberCommands.addOption("Deploy", new Deploy(true));

            // An option on elastic when pressed will reel the climber(is a boolean)
            ClimberCommands.addOption("Reel", new Reel(true));

            // The actually button with the features on elastic
            testTab.add("ClimberCommands", ClimberCommands)
                    .withSize(2, 1)
                    .withPosition(0, 5);

            // Making the last commands the selected ones
            lastSwerve = SwerveCommands.getSelected();
            lastArm = ArmCommands.getSelected();
            lastElevator = ElevatorCommands.getSelected();
            lastClimber = ClimberCommands.getSelected();
            lastIntake = IntakeCommands.getSelected();
            lastEndaffector = EndeffectorCommands.getSelected();
            lastAlgaeffector = AlgaeffectorCommands.getSelected();
        }
        testConfigured = true;

    }



    @Override
    public void testPeriodic() {

        if (SwerveCommands.getSelected() != lastSwerve) {
            SwerveCommands.getSelected().schedule();
        }

        if (ArmCommands.getSelected() != lastArm) {
            ArmCommands.getSelected().schedule();
        }
        if (ElevatorCommands.getSelected() != lastElevator) {
            ElevatorCommands.getSelected().schedule();
        }
        if (ClimberCommands.getSelected() != lastClimber) {
            ClimberCommands.getSelected().schedule();
        }
        if (IntakeCommands.getSelected() != lastIntake) {
            IntakeCommands.getSelected().schedule();
        }

        if (EndeffectorCommands.getSelected() != lastEndaffector) {
            EndeffectorCommands.getSelected().schedule();
        }
        if (AlgaeffectorCommands.getSelected() != lastAlgaeffector) {
            AlgaeffectorCommands.getSelected().schedule();
        }

        lastSwerve = SwerveCommands.getSelected();
        lastArm = ArmCommands.getSelected();
        lastElevator = ElevatorCommands.getSelected();
        lastClimber = ClimberCommands.getSelected();
        lastIntake = IntakeCommands.getSelected();
        lastEndaffector = EndeffectorCommands.getSelected();
        lastAlgaeffector = AlgaeffectorCommands.getSelected();

    unpublishTrajectory();

    }

    
   @Override
   public void testExit() {
     SwerveCommands.close();
     ArmCommands.close();
     ElevatorCommands.close();
     ClimberCommands.close();
     IntakeCommands.close();
     EndeffectorCommands.close();
 
   }





    @Override
    public void robotPeriodic() {
        SmartDashboard.putString("Scoring Enum", ScoringFactory.ScoreState.name());

        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        gscollect.update();
        teleopField.setRobotPose(drivetrain.getState().Pose);

        SmartDashboard.putString("Auto/Current Auto", autoName);

        CommandScheduler.getInstance().run();
    }

    @Override
    public void driverStationConnected() {

        DataLogManager.start(Constants.logDirectory);
        DataLogManager.log("Driverstation connected");
        DriverStation.startDataLog(DataLogManager.getLog());

        CommandScheduler.getInstance()
                .onCommandInitialize((action) -> DataLogManager.log(action.getName() + " Command Initialized"));
        CommandScheduler.getInstance()
                .onCommandInterrupt((action) -> DataLogManager.log(action.getName() + " Command Interrupted"));
        CommandScheduler.getInstance()
                .onCommandFinish((action) -> DataLogManager.log(action.getName() + " Command Finished"));
    }

    public void disabledInit() {
        setAllMotorsSafe();
    }

    @Override
    public void disabledPeriodic() {

        autoName = startChooser.getSelected() + firstPieceChooser.getSelected()
                + firstPieceRoLChooser.getSelected() + secondPieceChooser.getSelected() + secondPieceRoLChooser.getSelected()
                + thirdPieceChooser.getSelected() + thirdPieceRoLChooser.getSelected();

        if (!autoName.equals(oldAutoName)) {
            publishTrajectory(autoName);
            oldAutoName = autoName;
        }

    }

    @Override
    public void disabledExit() {

        if (autoName.equals("Pissing")) {

            drivetrain.resetRotation(Rotation2d.fromDegrees(isRed() ? -90 : 90));
            autoCommand = CommandSwerveDrivetrain.getInstance()
                    .applyRequest(() -> new SwerveRequest.FieldCentric().withVelocityX(
                    Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond) * -0.5))
                    .withTimeout(.8);

        } else if (autoName.equals("Shitting")) {
            drivetrain.resetRotation(Rotation2d.fromDegrees(isRed() ? -90 : 90));
            autoCommand = CommandSwerveDrivetrain.getInstance()
                    .applyRequest(() -> new SwerveRequest.FieldCentric().withVelocityX(
                    Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond) * -0.5))
                    .withTimeout(.75)
                    .andThen(ScoringFactory.L4Position().andThen(new Spit().withTimeout(0.5)).andThen(ScoringFactory.Stow()));

        } else if (!autoName.equals("")) {
            autoCommand = new PathPlannerAuto(autoName);
        } else {
            autoCommand = new PrintCommand("doing nothing!");
        }

        autoGroup = new SequentialCommandGroup(new Retract(true).withTimeout(0.2)

        );

        autoGroup.addCommands(
                new InstantCommand(timer::restart),
                new WaitUntilCommand(() -> timer.get() > pathDelay.getDouble(0)),
                autoCommand,
                new InstantCommand(timer::stop));

    }

    public void autonomousInit() {





        unpublishTrajectory();

        IntakePivot.getInstance().point3Intake();
        Climber.getInstance().zeroClimber();
        Elevator.getInstance().setHomePositionElevator();

        autoGroup.schedule();

    }


    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
 

      Climber.getInstance().setServoAngle(0);
        unpublishTrajectory();

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
   
        SmartDashboard.putNumber("ID Seen", LimelightHelpers.getFiducialID(Constants.VisionConstants.kLimelightFour));
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {

    }

    /**
     * A method to publish the trajectory of the autos
     *
     * @param autoName what auto you want to publish
     */
    public static void publishTrajectory(String autoName) {

        PathPlannerAuto auto = new PathPlannerAuto(autoName);
        if (autoName == null) {
            teleopField.getObject(Constants.SwerveConstants.AutoConstants.kFieldObjectName)
                    .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
            lastAutoName = "null";
            noAutoSelected.set(true);
            nullAuto.set(false);
            return;
        } // if we are calling publish trajectory but the trajectory is already published
        else if (autoName.equals(lastAutoName)) {
            return;
        }

        if (autoName.equals("Pissing")) {
            PissingAlert.set(true);
        } // we are going to use the auto name so this is the last auto we published
        else {
            lastAutoName = autoName;
        }

        List<Pose2d> poses = new ArrayList<>();
        poses.clear();

        try {
            // take the auto and then break it down into paths
            // and then from the paths break it down into Path points
            // and then we take the path poses from there
            PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach((path) -> path.getAllPathPoints()
                    .forEach((point) -> {
                        Pose2d pose = new Pose2d(point.position, point.position.getAngle());
                        if (isRed()) {
                            pose = FlippingUtil.flipFieldPose(pose);

                        }

                        poses.add(pose);
                    }));
            // flip the poses if we are red
            if (isRed()) {
                teleopField.getObject("Starting Pose")
                        .setPose(FlippingUtil.flipFieldPose(auto.getStartingPose()));
            } else {
                teleopField.getObject("Starting Pose")
                        .setPose(auto.getStartingPose());
            }

            // none of these are true so these alerts are usless
            nullAuto.set(false);
            publishfail.set(false);
            noAutoSelected.set(false);
            PissingAlert.set(false);
        } catch (RuntimeException e) {
            // if we call it and we have a null auto name when we are publishing it
            System.out.println("Null Auto: " + autoName);
            nullAuto.setText("Null auto: " + autoName);
            nullAuto.set(true);
        } catch (Exception e) {
            // if for some reason it completely dies
            publishfail.set(true);
            e.printStackTrace();
        }

        Robot.teleopField.getObject(Constants.SwerveConstants.AutoConstants.kFieldObjectName).setPoses(poses);
    }

    /**
     * a method that uses the {@code publishTrajectory} method and sets it to
     * null
     * <p>
     * if we want to unpublish trajectory we set auto name to null and we
     * publish a trajectory to a place where we can't see
     */
    public static void unpublishTrajectory() {
        publishTrajectory(null);
    }

    public static boolean isRed() {
        if (Robot.isSimulation()) {
            return false;
        }
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return true;
        } else {
            return false;
  }

        }
    

    public static void verifyMotors(TalonFX... falcons) {
        for (TalonFX falcon : falcons) {
            verifyMotor(falcon);
        }
    }

    public void setAllMotorsSafe() {
        Arm.getInstance().setArmDutyCycle(0);
        Elevator.getInstance().setElevatorDutyCycle(0);
        IntakePivot.getInstance().setPiviotDutyCycle(0);
        IntakeWheels.getInstance().runDutyCycleIntake(0);

    }

    /**
     * Will return false if the motor is verified and connected, true if there
     * is some error getting position
     *
     * @param falcon a TalonFX motor
     * @return false if the position is available, true if not available
     */
    public static boolean verifyMotor(TalonFX falcon) {

        StatusCode status = falcon.getPosition().getStatus();
        if (status.isError() && Robot.isReal()) {
            DataLogManager.log("TalonFX ID #" + falcon.getDeviceID() + " has failed to return position with status: "
                    + status.getDescription() + ". Error Code: " + status.value);
            MotorAlert.setText(
                    "TalonFX ID #" + falcon.getDeviceID() + " has failed to return position with status: "
                    + status.getName() + ". Error Code: ");
            MotorAlert.set(true);
            return true;
        }

        return false;
    }

    /**
     * Will return false if the CANcoder is verified and connected, true if
     * there is some error getting position
     *
     * @param coder a CANcoder
     * @return false if the position is available, true if not available
     */
    public static boolean verifyCANcoder(CANcoder coder) {
        StatusCode status = coder.getPosition().getStatus();
        if (status.isError() && Robot.isReal()) {
            DataLogManager.log("CANcoder ID #" + coder.getDeviceID() + " has failed to return position with status: "
                    + status.getDescription() + ". Error Code: " + status.value);
            CANcoderAlert.setText(
                    "CANcoder ID #" + coder.getDeviceID() + " has failed to return position with status: "
                    + status.getName() + ". Error Code: " + status.value);
            CANcoderAlert.set(true);
            return true;
        }

        return false;
    }

    private static final class GcStatsCollector {

        private List<GarbageCollectorMXBean> gcBeans = ManagementFactory.getGarbageCollectorMXBeans();
        private MemoryMXBean memBean = ManagementFactory.getMemoryMXBean();
        private final long[] lastTimes = new long[gcBeans.size()];
        private final long[] lastCounts = new long[gcBeans.size()];

        public void update() {
            long accumTime = 0;
            long accumCounts = 0;
            for (int i = 0; i < gcBeans.size(); i++) {
                long gcTime = gcBeans.get(i).getCollectionTime();
                long gcCount = gcBeans.get(i).getCollectionCount();
                accumTime += gcTime - lastTimes[i];
                accumCounts += gcCount - lastCounts[i];

                lastTimes[i] = gcTime;
                lastCounts[i] = gcCount;
            }

            SmartDashboard.putNumber("Memory/GC Time MS", (double) accumTime);
            SmartDashboard.putNumber("Memory/GCCounts", (double) accumCounts);
            SmartDashboard.putNumber("Memory/Usage", (double) memBean.getHeapMemoryUsage().getUsed());

            if (accumTime > (20)) {
                gcAlert.set(true);
            } else {
                gcAlert.set(false);

            }
        }
    }

}
