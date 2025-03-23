package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Endeffector extends SubsystemBase {
    private static Endeffector mInstance;
    
    private TalonSRX endeffectorMotor;

    public static Endeffector getInstance() {
        if (mInstance == null)
            mInstance = new Endeffector();
        return mInstance;
    }

    public Endeffector() {
        endeffectorMotor = new TalonSRX(Constants.EndeffectorConstants.kEndeffectorMotorId);
    }

    public void setEneffdector(double endeffectorDutyCycle) {
        endeffectorMotor.set(TalonSRXControlMode.PercentOutput, endeffectorDutyCycle); 
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Endeffector/status", status);

    }
    public void setEndeffectorBrake() {
        endeffectorMotor.setNeutralMode(NeutralMode.Brake);
    }
    


    @Override
    public void periodic() {

        SmartDashboard.putNumber("Speed Endeffector", endeffectorMotor.getMotorOutputPercent());
    }

}
