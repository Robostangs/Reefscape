package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Endeffector extends SubsystemBase {
    private static Endeffector mInstance;
    private TalonFX endeffectorMotorRight, endeffectorMotorLeft;
    private DigitalInput EndeffectorSensor;

    public static Endeffector getInstance() {
        if (mInstance == null)
            mInstance = new Endeffector();
        return mInstance;
    }

    // just spit and put in break mode
    public Endeffector() {
        endeffectorMotorRight = new TalonFX(Constants.EndeffectorConstants.kEndeffectorRightMotorId);
        endeffectorMotorLeft = new TalonFX(Constants.EndeffectorConstants.kEndeffectorRightMotorId);
        // EndeffectorSensor = new DigitalInput(Constants.EndeffectorConstants.kEndeffectorLeftMotorId);

    }

    public void setEneffdector(double endeffectorDutyCycle) {
        endeffectorMotorRight.set(endeffectorDutyCycle);
        endeffectorMotorLeft.set(endeffectorDutyCycle);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Endeffector/status", status);

    }
    public void setEndeffectorBrake() {
        endeffectorMotorRight.setNeutralMode(NeutralModeValue.Brake);
        endeffectorMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    }
    
    public boolean getEndeffectorSensor() {
        return EndeffectorSensor.get();
    }

    @Override
    public void periodic() {
        // TODO add logging

    }

}
