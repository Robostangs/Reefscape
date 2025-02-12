package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Endeffector extends SubsystemBase {
    private static Endeffector mInstance;
    
    private TalonSRX endeffectorMotorRight;
    private DigitalInput EndeffectorSensor;

    public static Endeffector getInstance() {
        if (mInstance == null)
            mInstance = new Endeffector();
        return mInstance;
    }

    // just spit and put in break mode
    public Endeffector() {
        endeffectorMotorRight = new TalonSRX(Constants.EndeffectorConstants.kEndeffectorRightMotorId);
        EndeffectorSensor = new DigitalInput(Constants.EndeffectorConstants.kEndeffectorSensorId);

    }

    public void setEneffdector(double endeffectorDutyCycle) {
        endeffectorMotorRight.set(TalonSRXControlMode.PercentOutput, endeffectorDutyCycle); 
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Endeffector/status", status);

    }
    public void setEndeffectorBrake() {
        endeffectorMotorRight.setNeutralMode(NeutralMode.Brake);
    }
    
    public boolean getEndeffectorSensor() {
        return EndeffectorSensor.get();
    }

    @Override
    public void periodic() {
        // TODO add logging

    }

}
