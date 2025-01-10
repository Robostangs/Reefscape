package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Elevator {
    private static Elevator mInstance;
    private TalonFX elevatorMotor;
    private double elevatorPosition;

    public static Elevator getInstance() {
        if (mInstance == null)
            mInstance = new Elevator();
        return mInstance;
    }

    public Elevator() {
        elevatorMotor = new TalonFX(Constants.ElevatorConstants.kElevatorMotorId);

        var slot0Configs = new Slot0Configs();
        // TODO tune these values
        slot0Configs.kP = Constants.ElevatorConstants.kElevatorP;
        slot0Configs.kI = Constants.ElevatorConstants.kElevatorI;
        slot0Configs.kD = Constants.ElevatorConstants.kElevatorD;
        slot0Configs.kS = Constants.ElevatorConstants.kElevatorFF;

        elevatorMotor.getConfigurator().apply(slot0Configs);

    }

    public void periodic() {
        SmartDashboard.putNumber("Elevator/position", elevatorPosition);
    }

    public void setElevatorPosition(double rotations) {
        if (rotations > Constants.ElevatorConstants.kminElevatorHeight
                && rotations < Constants.ElevatorConstants.kmaxElevatorHeight) {
            elevatorMotor.setPosition(rotations);
            elevatorPosition = rotations;
        }

    }

    public double getElevatorPosition() {
        return elevatorPosition;
    }

}