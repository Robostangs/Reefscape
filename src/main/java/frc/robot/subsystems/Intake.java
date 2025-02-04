package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotorTop,piviotMotor;
    private Alert intakeAlert = new Alert("INTAKE TWEAKING", Alert.AlertType.kError);
    private static Intake mInstance;
    private DigitalInput IntakeSensor;

    public static Intake getInstance() {
        if (mInstance == null)
            mInstance = new Intake();
        return mInstance;
    }

    public Intake() {

        var talonFXConfigs = new TalonFXConfiguration();

        intakeMotorTop = new TalonFX(Constants.IntakeConstants.kTopIntakeMotorId);
        piviotMotor = new TalonFX(Constants.IntakeConstants.kBarMotorId);
        IntakeSensor = new DigitalInput(Constants.IntakeConstants.kIntakeSensorId);
        
        intakeMotorTop.getConfigurator().apply(talonFXConfigs);
        piviotMotor.getConfigurator().apply(talonFXConfigs);

    }

    public void runIntake(double IntakeDutyCycle) {
        intakeMotorTop.set(IntakeDutyCycle);
    }

    public void extendBar() {
        piviotMotor.set(.5);
    }

    public void retractBar() {
        piviotMotor.set(-.5);

    }

    public void postStatus(String status) {
        SmartDashboard.putString("Intake/status", status);

    }

    public void stopIntake() {
        intakeMotorTop.stopMotor();
    }

    public boolean getIntakeSensor() {
        return IntakeSensor.get();
    }
    public void setIntakePiviotBrake() {
        piviotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        // TODO add logging

    }

    // big b at your service
    // -Bardia
}
