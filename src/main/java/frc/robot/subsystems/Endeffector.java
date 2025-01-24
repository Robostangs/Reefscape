package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Endeffector extends SubsystemBase {
    private static Endeffector mInstance;
    private TalonFX endefectorPivoitMotor;
    private TalonFX endefectorMotor;

    public static Endeffector getInstance() {
        if (mInstance == null)
            mInstance = new Endeffector();
        return mInstance;
    }

    //just spit and put in break mode
    public Endeffector() {
        endefectorPivoitMotor = new TalonFX(Constants.EndefectorConstants.kEndefectorPiviotMotorId);
        endefectorMotor = new TalonFX(Constants.EndefectorConstants.kEndefectorMotorId);    

        

    }

    @Override
    public void periodic() {
        // TODO add logging

    }

}
