package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm mInstance;

    public static Arm getInstance() {
        if (mInstance == null)
            mInstance = new Arm();
        return mInstance;
    }

    public Arm() {
    }

    @Override
    public void periodic() {
        // TODO add logging

    }
    
}
