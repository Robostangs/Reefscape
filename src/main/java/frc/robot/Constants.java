// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Endefector;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String logDirectory = "";

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipControllerPort = 1;

  }

  public static class IntakeConstants {
    public static final int kTopIntakeMotorId = 0;
    public static final int kBottomIntakeMotorId = 0;
    public static final int kBarMotorId = 0;

    public static final boolean kTopIntakeMotorInverted = false;
    public static final boolean kBottomIntakeMotorInverted = false;
  }
  public static class EndefectorConstants{

    public static final int kEndefectorLeftMotorId = 0;
    public static final int kEndefectorRightMotorId = 0;

  }

  public static class ElevatorConstants {
    public static final int kElevatorMotorId = 0;
    public static final double kElevatorP = 0;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorFF = 0;
    public static final double kElevatorMaxCurrent = 40.0d;
    //TODO FINNNNDDDD ALLLLL THESSESEESE F******NG VALUES
    public static final double kmaxElevatorHeight = 10d;
    public static final double kminElevatorHeight = 0d;
    public static final double kRotationstoMeters = 1d;
    public static final double kElevatorWeight = 5d;
    public static final double kElevatorHeight = 1d;
    public static final double kElevatorWidth = 1d;
    public static final double kRootElevatorX = .3;
    public static final double kRootElevatorY = 0d;
    public static final double kRootElevator2X = .6;
    public static final double kRootElevator2Y = 0d;
    public static final double kLigaLength = .3d;
    public static final double kElevatorGearing = 0d;
    public static final double kDrumRadius = 0d;
    



  }

}
