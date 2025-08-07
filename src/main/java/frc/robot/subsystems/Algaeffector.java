package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Algaeffector extends SubsystemBase {
    private TalonFXS algaeman;

    private static Algaeffector mInstance;

    public Algaeffector(){
      algaeman = new TalonFXS(Constants.AlgaeffectorConstants.kalgaeffectorid);
    }

    public static Algaeffector getInstance() {
        if (mInstance == null)
            mInstance = new Algaeffector();
        return mInstance;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Algae/status", status);

    }

    public void periodic(){
        SmartDashboard.putNumber("AlgaeffectorDutyCycle", algaeman.getDutyCycle().getValueAsDouble());
    }

    public void setEneffdector(double kalgaeffectordutycyle) {
        
        algaeman.set(kalgaeffectordutycyle);
    }

    public void setEndeffectorBrake(){
        algaeman.setNeutralMode(NeutralModeValue.Brake);
    }
    
}
