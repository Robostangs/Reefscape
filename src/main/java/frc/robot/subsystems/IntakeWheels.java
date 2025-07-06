package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWheels extends SubsystemBase {
    private TalonFX intakeMotor;
    private static IntakeWheels mInstance;
    private DigitalInput IntakeSensor;

    public static IntakeWheels getInstance() {
        if (mInstance == null)
            mInstance = new IntakeWheels();
        return mInstance;
    }

     

    public IntakeWheels() {

        intakeMotor = new TalonFX(Constants.IntakeConstants.kWheelMotorId);
        IntakeSensor = new DigitalInput(Constants.IntakeConstants.kIntakeSensorId);

    }

    public void runDutyCycleIntake(double IntakeDutyCycle) {
        intakeMotor.set(IntakeDutyCycle);
    }

    public void zeroIntake() {
        intakeMotor.setPosition(0d);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Intake/status", status);

    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public boolean getIntakeSensor() {
    // Returns true if the intake sensor is triggered (object detected).
        return IntakeSensor.get();
    }



}
