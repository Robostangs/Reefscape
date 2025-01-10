package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake {
    private TalonFX intakeMotortop, intakeMotorbottom;
    private Alert intakeAlert = new Alert("INTAKE TWEAKING", Alert.AlertType.kError);
    private static Intake mInstance;

    public static Intake getInstance() {
        if (mInstance == null)
            mInstance = new Intake();
        return mInstance;
    }

    public Intake() {

        var talonFXConfigs = new TalonFXConfiguration();

        intakeMotortop = new TalonFX(Constants.IntakeConstants.kTopIntakeMotorId);
        intakeMotorbottom = new TalonFX(Constants.IntakeConstants.kBottomIntakeMotorId);



    
    }

    public void runIntake(double IntakeDutyCycle) {
        intakeMotortop.set(IntakeDutyCycle);
        intakeMotorbottom.set(IntakeDutyCycle);
    }

    public void stopIntake() {
        intakeMotortop.stopMotor();
        intakeMotorbottom.stopMotor();
    }

    public void periodic() {
        //TODO add logging

    }
}
