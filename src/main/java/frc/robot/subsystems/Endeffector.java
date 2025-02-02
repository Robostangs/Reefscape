package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Endeffector extends SubsystemBase {
    private static Endeffector mInstance;
    private TalonFX endefectorMotorRight, endefectorMotorLeft;
    private DigitalInput EndefectorSensor;

    public static Endeffector getInstance() {
        if (mInstance == null)
            mInstance = new Endeffector();
        return mInstance;
    }

    // just spit and put in break mode
    public Endeffector() {
        endefectorMotorRight = new TalonFX(Constants.EndefectorConstants.kEndefectorRightMotorId);
        endefectorMotorLeft = new TalonFX(Constants.EndefectorConstants.kEndefectorRightMotorId);
        EndefectorSensor = new DigitalInput(Constants.EndefectorConstants.kEndefectorSensorId);

    }

    public void setEneffdector(double eneffectorDutyCycle) {
        endefectorMotorRight.set(eneffectorDutyCycle);
        endefectorMotorLeft.set(eneffectorDutyCycle);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Endeffector/status", status);

    }
    
    public boolean getEndefectorSensor() {
        return EndefectorSensor.get();
    }

    @Override
    public void periodic() {
        // TODO add logging

    }

}
