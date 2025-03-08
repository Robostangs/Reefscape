// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.HomeElevator;
import frc.robot.commands.EndeffectorCommands.Slurp;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.commands.Factories.IntakeFactory;
import frc.robot.commands.Factories.ScoringFactory;
import frc.robot.commands.IntakeCommands.Retract;
import frc.robot.commands.SwerveCommands.PathToPoint;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeWheels;

public class Robot extends TimedRobotstangs {

  private final RobotContainer m_robotContainer;

  private CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

  public static Field2d teleopField = new Field2d();

  public static ShuffleboardTab autoTab, teleopTab, testTab, disTab;

  private static Alert gcAlert = new Alert("MEMORY TWEAKING FIX RN", AlertType.kError);
  private static Alert CANcoderAlert = new Alert("Can tweaking", AlertType.kError);
  private static Alert MotorAlert = new Alert("Can tweaking", AlertType.kError);

  private static String autoName = "";

  // Autos
  private SendableChooser<String> startChooser = new SendableChooser<>();
  private SendableChooser<String> firstPieceChooser = new SendableChooser<>();
  private SendableChooser<String> firstPieceRoLChooser = new SendableChooser<>();
  private SendableChooser<String> secondPieceChooser = new SendableChooser<>();
  private SendableChooser<String> secondPieceRoLChooser = new SendableChooser<>();
  private SendableChooser<String> thirdPieceChooser = new SendableChooser<>();
  private SendableChooser<String> thirdPieceRoLChooser = new SendableChooser<>();

  private Command autoCommand;

  private static GcStatsCollector gscollect = new GcStatsCollector();
  private static String lastAutoName;
  private static Alert nullAuto = new Alert("Null auto", AlertType.kWarning);
  private static Alert publishfail = new Alert("Publishing failed", AlertType.kError);
  private static Alert noAutoSelected = new Alert("No Auto Selected", AlertType.kWarning);
  private static Alert ShittyAlert = new Alert("We going forward ", AlertType.kInfo);

  private String oldAutoName = "";

  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void robotInit() {

    SmartDashboard.putData("Field", teleopField);
    teleopTab = Shuffleboard.getTab("Teleoperated");
    autoTab = Shuffleboard.getTab("Autonomous");
    testTab = Shuffleboard.getTab("Test");
    disTab = Shuffleboard.getTab("Disabled");

    startChooser.setDefaultOption("Shit and Shit", "");
    startChooser.addOption("Forward", "shitting");
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

    // autoTab.add("", Alert.class.get);
    autoName = startChooser.getSelected() + firstPieceChooser.getSelected() + firstPieceRoLChooser.getSelected()
        + secondPieceChooser.getSelected() + secondPieceRoLChooser.getSelected()
        + thirdPieceChooser.getSelected() + thirdPieceRoLChooser.getSelected();

    NamedCommands.registerCommand("L1 Prime", new PrintCommand("Hello!"));
    NamedCommands.registerCommand("L2 Prime", new PrintCommand("Hello!"));
    NamedCommands.registerCommand("L3 Prime", ScoringFactory.L3Position().withTimeout(1.5));
    NamedCommands.registerCommand("L4 Prime", ScoringFactory.L4Position().withTimeout(1.5));
    // TODO change this to spit
    NamedCommands.registerCommand("Spit", new Spit().withTimeout(1.5));
    NamedCommands.registerCommand("Slurp", new Slurp().withTimeout(1.5));


    NamedCommands.registerCommand("Feeder Intake", IntakeFactory.SourceIntake());
    NamedCommands.registerCommand("Return Home", ScoringFactory.Stow().withTimeout(1.5));
    // TODO add a delay to path

    LiveWindow.enableAllTelemetry();
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
    // TODO ake the motors nuetral or something so they dont go back to their
    // setpoints

    setAllMotorsSafe();
  }

  @Override
  public void disabledPeriodic() {

    autoName = startChooser.getSelected() + firstPieceChooser.getSelected() +
        firstPieceRoLChooser.getSelected() + secondPieceChooser.getSelected() + secondPieceRoLChooser.getSelected()
        + thirdPieceChooser.getSelected() + thirdPieceRoLChooser.getSelected();

    if (!autoName.equals(oldAutoName)) {
      publishTrajectory(autoName);
      oldAutoName = autoName;
    }

    // teleopField.getObject("Starting
    // Pose").setPose(Constants.SwerveConstants.AutoConstants.AutoPoses.kCenterPose);
  }

  public void autonomousInit() {
    unpublishTrajectory();
    
    IntakePivot.getInstance().point3Intake();
    Climber.getInstance().zeroClimber();
    SequentialCommandGroup autoGroup = new SequentialCommandGroup(new Retract().withTimeout(0.2),new HomeElevator().withTimeout(1.5));


    if (autoName.equals("shitting")) {
      // TODO do the shit with the shit

      drivetrain.resetRotation(Rotation2d.fromDegrees(isRed() ? 180 : 0));
      autoCommand = CommandSwerveDrivetrain.getInstance()
          .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(
              Constants.SwerveConstants.AutoConstants.AutoSpeeds.kSpeedAt12Volts.in(MetersPerSecond) * 0.15))
          .withTimeout(1d);

    } else if (autoName.equals("PTP")) {
      autoCommand = new PathToPoint(!isRed() ? Constants.ScoringConstants.k21BlueRReefPosePtP
          : FlippingUtil.flipFieldPose(Constants.ScoringConstants.k21BlueRReefPosePtP))
          .andThen(ScoringFactory.L3Score());
    } else if (!autoName.equals("")) {
      autoCommand = new PathPlannerAuto(autoName);
    } else {
      autoCommand = new PrintCommand("doing nothing!");
    }


    autoGroup.addCommands(
        autoCommand);

    autoGroup.schedule();

    // autoCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // only in pits
    // if (!DriverStation.isFMSAttached()) {
    // ScoringFactory.Stow().schedule();
    // }

    unpublishTrajectory();

    drivetrain.resetRotation(Rotation2d.fromDegrees(isRed() ? 0 : 180));

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
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
    }

    // if we are calling publish trajectory but the trajectory is already published
    else if (autoName.equals(lastAutoName)) {
      return;
    }

    if (autoName.equals("shitting")) {
      ShittyAlert.set(true);
    }
    // we are going to use the auto name so this is the last auto we published
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
      ShittyAlert.set(false);
    } catch (RuntimeException e) {
      // if we call it and we have a null auto name when we are publishing it
      System.out.println("Null Auto: " + autoName);
      nullAuto.setText("Null auto: " + autoName);
      nullAuto.set(true);
    }

    catch (Exception e) {
      // if for some reason it completely dies
      publishfail.set(true);
      e.printStackTrace();
    }

    Robot.teleopField.getObject(Constants.SwerveConstants.AutoConstants.kFieldObjectName).setPoses(poses);
  }

  /**
   * a method that uses the {@code publishTrajectory} method and sets it to null
   * <p>
   * if we want to unpublish trajectory we set auto name to null and we publish a
   * trajectory to a place where we can't see
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

  public void setAllMotorsSafe(){
    Arm.getInstance().setArmDutyCycle(0);
    Elevator.getInstance().setElevatorDutyCycle(0);
    IntakePivot.getInstance().setPiviotDutyCycle(0);
    IntakeWheels.getInstance().runDutyCycleIntake(0);

  }

  /**
   * Will return false if the motor is verified and connected, true if there is
   * some error getting position
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
   * Will return false if the CANcoder is verified and connected, true if there is
   * some error getting position
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
