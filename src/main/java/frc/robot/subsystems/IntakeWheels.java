package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWheels extends SubsystemBase {
    private TalonFX intakeMotor;
    private Alert intakeAlert = new Alert("INTAKE TWEAKING", Alert.AlertType.kError);
    private static IntakeWheels mInstance;
    private DigitalInput IntakeSensor;

    public static IntakeWheels getInstance() {
        if (mInstance == null)
            mInstance = new IntakeWheels();
        return mInstance;
    }

    public IntakeWheels() {

        intakeMotor = new TalonFX(Constants.IntakeConstants.kWheelMotorId);
        // IntakeSensor = new DigitalInput(Constants.IntakeConstants.kIntakeSensorId);

    }





    public void runIntake(double IntakeDutyCycle) {
        intakeMotor.set(IntakeDutyCycle);
    }

    public void zeroIntake() {
        intakeMotor.setPosition(0d);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Intake/status", status);

    }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    public boolean getIntakeSensor() {
        return IntakeSensor.get();
    }


    
    @Override
    public void periodic() {

        SmartDashboard.putBoolean("Intake Senor", getIntakeSensor());

    }

}
