// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class Robot extends TimedRobotstangs {
  public XboxController xDrive = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
  public XboxController xManip = new XboxController(Constants.OperatorConstants.kManipControllerPort);

  private final RobotContainer m_robotContainer;

  private Elevator elevator = Elevator.getInstance();
  private Arm arm = Arm.getInstance();

  public static ShuffleboardTab autoTab, teleopTab, testTab;

  private static Alert gcAlert = new Alert("MEMORY TWEAKING FIX RN", Alert.AlertType.ERROR);

  //Autos
  private SendableChooser<String> startChooser = new SendableChooser<>();
  private SendableChooser<String> firstPieceChooser = new SendableChooser<>();
  private SendableChooser<String> firstPieceRoLChooser = new SendableChooser<>();
  private SendableChooser<String> secondPieceChooser = new SendableChooser<>();
  private SendableChooser<String> secondPieceRoLChooser = new SendableChooser<>();
  private SendableChooser<String> thirdPieceChooser = new SendableChooser<>();
  private SendableChooser<String> thirdPieceRoLChooser = new SendableChooser<>();

  private Command autoCommand;


  

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

    teleopTab = Shuffleboard.getTab("Teleoperated");
		autoTab = Shuffleboard.getTab("Autonomous");
		testTab = Shuffleboard.getTab("Test");


    startChooser.setDefaultOption("Center", "Cstart");
    startChooser.addOption("Open", "Ostart");
    startChooser.addOption("Processor", "Pstart");

    firstPieceChooser.setDefaultOption("Center 1", "C1");
    firstPieceChooser.setDefaultOption("Center 2", "C2");
    firstPieceChooser.setDefaultOption("Pro 1", "P1");
    firstPieceChooser.setDefaultOption("Pro 2", "P2");
    firstPieceChooser.setDefaultOption("Open 1", "O1");
    firstPieceChooser.setDefaultOption("Open 2", "O2");

    firstPieceRoLChooser.setDefaultOption("Right", "R");
    firstPieceRoLChooser.setDefaultOption("Left", "L");

    secondPieceChooser.setDefaultOption("Center 1", "C1");
    secondPieceChooser.setDefaultOption("Center 2", "C2");
    secondPieceChooser.setDefaultOption("Pro 1", "P1");
    secondPieceChooser.setDefaultOption("Pro 2", "P2");
    secondPieceChooser.setDefaultOption("Open 1", "O1");
    secondPieceChooser.setDefaultOption("Open 2", "O2");

    secondPieceRoLChooser.setDefaultOption("Right", "Right");
    secondPieceRoLChooser.setDefaultOption("Left", "Left");

    thirdPieceChooser.setDefaultOption("Center 1", "C1");
    thirdPieceChooser.setDefaultOption("Center 2", "C2");
    thirdPieceChooser.setDefaultOption("Pro 1", "P1");
    thirdPieceChooser.setDefaultOption("Pro 2", "P2");
    thirdPieceChooser.setDefaultOption("Open 1", "O1");
    thirdPieceChooser.setDefaultOption("Open 2", "O2");

    thirdPieceRoLChooser.setDefaultOption("Right", "Right");
    thirdPieceRoLChooser.setDefaultOption("Left", "Left");





    Alert.groups.forEach((group, alert) -> {
      autoTab.add(group, alert)
          .withSize(3, 3)
          .withPosition(6, 1)
          .withWidget("Alerts");
    });


			Alert.groups.forEach((group, alert) -> {
				testTab.add(group, alert)
						.withSize(2, 3)
						.withPosition(0, 0)
						.withWidget("Alerts");
			});


			Alert.groups.forEach((group, alert) -> {
				teleopTab.add(group, alert)
						.withSize(2, 3)
						.withPosition(0, 0)
						.withWidget("Alerts");
			});
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
  }

  public void autonomousInit() {

    //TODO add right or left somehow

    autoCommand = AutoBuilder.buildAuto(startChooser.getSelected()+" - "+
     firstPieceChooser.getSelected()+firstPieceRoLChooser.getSelected()+" - "+
     secondPieceChooser.getSelected()+ secondPieceRoLChooser.getSelected()+" - "+
        thirdPieceChooser.getSelected()+ thirdPieceRoLChooser.getSelected());

 

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
