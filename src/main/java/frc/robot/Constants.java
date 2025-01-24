// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.util.Units;

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

    public static final int kEndefectorPiviotMotorId = 0;
    public static final int kEndefectorMotorId = 0;


  }

  public static class ArmConstants{
    public static final int kArmMotorId = 30;
    public static final double kArmP = 0; 
    public static final double kArmI = 0;
    public static final double kArmD = 0;
    public static final double kArmFF = 0;
    public static final GravityTypeValue kArmgravtype= GravityTypeValue.Arm_Cosine;
    public static final int kArmEncoderId = 12;
    public static final double kArmheight = 5d;
    public static final double kArmWidth = 5d;
    public static final double kArmRootX = 1d;
    public static final double kArmRootY = 2d;

    

  }

  public static class ElevatorConstants {
    public static final int kElevatorMotorId = 2;
    public static final double kElevatorP = 5;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorFF = 1;
    public static final double kElevatorMaxCurrent = 40.0d;
    public static final int kElevatorEncoderId = 3;
    //TODO FINNNNDDDD ALLLLL THESSESEESE F******NG VALUES
    public static final double kmaxElevatorHeight = 20d;
    public static final double kMinElevatorHeight = 0d;
    public static final double kRotationstoMeters = 1d;
    public static final double kElevatorWeight = 24d;
    public static final double kElevatorHeight = 1d;
    public static final double kElevatorWidth = 1d;
    public static final double kRootElevatorX = .3;
    public static final double kRootElevatorY = 0d;
    public static final double kRootElevator2X = .6;
    public static final double kRootElevator2Y = 0d;
    public static final double kLigaLength = .3d;
    public static final double kElevatorGearing = 8d;
    public static final double kDrumRadius = Units.inchesToMeters(2.0);;

  }

}
