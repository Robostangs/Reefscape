package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Endefector extends SubsystemBase {
    private static Endefector mInstance;
    private TalonFX endefectorPivoitMotor;
    private TalonFX endefectorMotor;

    public static Endefector getInstance() {
        if (mInstance == null)
            mInstance = new Endefector();
        return mInstance;
    }

    public Endefector() {
        endefectorPivoitMotor = new TalonFX(Constants.EndefectorConstants.kEndefectorPiviotMotorId);
        endefectorMotor = new TalonFX(Constants.EndefectorConstants.kEndefectorMotorId);    

        

    }

    @Override
    public void periodic() {
        // TODO add logging

    }

}
