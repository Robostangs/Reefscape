// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobotstangs {
  public XboxController xDrive = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
  public XboxController xManip = new XboxController(Constants.OperatorConstants.kManipControllerPort);

  private final RobotContainer m_robotContainer;

  private Elevator elevator = Elevator.getInstance();
  private Arm arm = Arm.getInstance();
  private CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

  public static Field2d teleopField = new Field2d();

  public static ShuffleboardTab autoTab, teleopTab, testTab, disTab;

  private static Alert gcAlert = new Alert("MEMORY TWEAKING FIX RN", Alert.AlertType.kError);

  private static String autoName = "";

  // Autos
  private SendableChooser<String> startChooser = new SendableChooser<>();
  private SendableChooser<String> firstPieceChooser = new SendableChooser<>();
  private SendableChooser<String> firstPieceRoLChooser = new SendableChooser<>();
  private SendableChooser<String> secondPieceChooser = new SendableChooser<>();
  private SendableChooser<String> secondPieceRoLChooser = new SendableChooser<>();
  private SendableChooser<String> thirdPieceChooser = new SendableChooser<>();
  private SendableChooser<String> thirdPieceRoLChooser = new SendableChooser<>();

  private PathPlannerAuto autoCommand;

  private static String lastAutoName;
  private static Alert nullAuto = new Alert("Null auto", AlertType.kWarning);
  private static Alert publishfail = new Alert("Publishing failed", AlertType.kError);
  private static Alert noAutoSelected = new Alert("No Auto Selected", AlertType.kWarning);

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

    startChooser.setDefaultOption("Center", "CStart");
    startChooser.addOption("Open", "OStart");
    startChooser.addOption("Processor", "PStart");

    firstPieceChooser.setDefaultOption("Center 1", " - C1");
    firstPieceChooser.addOption("Center 2", " - C2");
    firstPieceChooser.addOption("Pro 1", " - P1");
    firstPieceChooser.addOption("Pro 2", " - P2");
    firstPieceChooser.addOption("Open 1", " - O1");
    firstPieceChooser.addOption("Open 2", " - O2");

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
    Intake.getInstance().setPiviotZero();
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

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

  }

  @Override
  public void disabledPeriodic() {

    publishTrajectory(autoName);
  }

  public void autonomousInit() {

    autoCommand = new PathPlannerAuto(autoName);

    switch (startChooser.getSelected()) {
      case "CStart":
        CommandSwerveDrivetrain.getInstance().resetPose(DriverStation.Alliance.Blue == DriverStation.getAlliance().get()
            ? Constants.SwerveConstants.AutoConstants.AutoPoses.kCenterPose
            : FlippingUtil.flipFieldPose(Constants.SwerveConstants.AutoConstants.AutoPoses.kCenterPose)
                .rotateBy(new Rotation2d(180)));

        break;
      case "OStart":
        CommandSwerveDrivetrain.getInstance().resetPose(DriverStation.Alliance.Blue == DriverStation.getAlliance().get()
            ? Constants.SwerveConstants.AutoConstants.AutoPoses.kOpenPose
            : FlippingUtil.flipFieldPose(Constants.SwerveConstants.AutoConstants.AutoPoses.kOpenPose)
                .rotateBy(new Rotation2d(180)));

        break;

      case "PStart":
        CommandSwerveDrivetrain.getInstance().resetPose(DriverStation.Alliance.Blue == DriverStation.getAlliance().get()
            ? Constants.SwerveConstants.AutoConstants.AutoPoses.kProPose
            : FlippingUtil.flipFieldPose(Constants.SwerveConstants.AutoConstants.AutoPoses.kProPose)
                .rotateBy(new Rotation2d(180)));

        break;

      default:
        drivetrain.resetPose(drivetrain.getState().Pose);

        break;
    }

    autoCommand.schedule();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
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
    // else if (autoName.equals(lastAutoName)) {
    // return;
    // }

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
            if (DriverStation.getAlliance().get() == Alliance.Red) {
              pose = FlippingUtil.flipFieldPose(pose);

            }

            poses.add(pose);
          }));
      // flip the poses if we are red
      if (DriverStation.getAlliance().get() == Alliance.Red) {
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
    }

    catch (RuntimeException e) {
      // if we call it and we have a null auto name when we are publishing it
      System.out.println("Null Auto: " + autoName);
      nullAuto.setText("Null auto: " + autoName);
      nullAuto.set(true);
    }

    catch (Exception e) {
      // if for some reason it completly dies
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

      // TODO remake alerts
      if (accumTime > (20)) {
        gcAlert.set(true);
      } else {
        gcAlert.set(false);

      }

    }
  }

}
