package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Endeffector extends SubsystemBase {
    private static Endeffector mInstance;
    private TalonFX endefectorMotorRight,endefectorMotorLeft;

    public static Endeffector getInstance() {
        if (mInstance == null)
            mInstance = new Endeffector();
        return mInstance;
    }

    //just spit and put in break mode
    public Endeffector() {
        endefectorMotorRight = new TalonFX(Constants.EndefectorConstants.kEndefectorRightMotorId);   
        endefectorMotorLeft = new TalonFX(Constants.EndefectorConstants.kEndefectorRightMotorId); 

     
    }

    public void setEneffector(double eneffectorDutyCycle) {
        endefectorMotorRight.set(eneffectorDutyCycle);
        endefectorMotorLeft.set(eneffectorDutyCycle);
    }



    @Override
    public void periodic() {
        // TODO add logging

    }

}
