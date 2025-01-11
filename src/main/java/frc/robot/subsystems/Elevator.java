package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DifferentialPositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private static Elevator mInstance;
    private TalonFX elevatorMotor;
    private double elevatorPosition;
    private TorqueCurrentFOC elevatorControl;

    public static Elevator getInstance() {
        if (mInstance == null)
            mInstance = new Elevator();
        return mInstance;
    }

    public Elevator() {
        elevatorMotor = new TalonFX(Constants.ElevatorConstants.kElevatorMotorId);
        elevatorControl = new TorqueCurrentFOC(Constants.ElevatorConstants.kElevatorMaxCurrent);

        var slot0Configs = new Slot0Configs();
        // TODO tune these values
        slot0Configs.kP = Constants.ElevatorConstants.kElevatorP;
        slot0Configs.kI = Constants.ElevatorConstants.kElevatorI;
        slot0Configs.kD = Constants.ElevatorConstants.kElevatorD;
        slot0Configs.kS = Constants.ElevatorConstants.kElevatorFF;

        elevatorMotor.getConfigurator().apply(slot0Configs);
        
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/position", elevatorPosition);
    }

    public void setElevatorPosition(double TargetElevatorMeters) {
        if (TargetElevatorMeters > Constants.ElevatorConstants.kminElevatorHeight
                && TargetElevatorMeters < Constants.ElevatorConstants.kmaxElevatorHeight) {

            double elevatorrots = TargetElevatorMeters * Constants.ElevatorConstants.kRotationstoMeters;
            elevatorMotor.setPosition(elevatorrots);
            elevatorPosition = TargetElevatorMeters;
        }

    }

    public double getElevatorPosition() {
        return elevatorPosition;
    }

}