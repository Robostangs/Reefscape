package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Endefector extends SubsystemBase {
    private static Endefector mInstance;

    public static Endefector getInstance() {
        if (mInstance == null)
            mInstance = new Endefector();
        return mInstance;
    }

    public Endefector() {
    }

    @Override
    public void periodic() {
        // TODO add logging

    }

}
