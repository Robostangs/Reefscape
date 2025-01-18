package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotortop, intakeMotorbottom, barMotor;
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
        barMotor = new TalonFX(Constants.IntakeConstants.kBarMotorId);

        intakeMotortop.getConfigurator().apply(talonFXConfigs);
        intakeMotorbottom.getConfigurator().apply(talonFXConfigs);
        barMotor.getConfigurator().apply(talonFXConfigs);

    }

    public void runIntake(double IntakeDutyCycle) {
        intakeMotortop.set(IntakeDutyCycle);
        intakeMotorbottom.set(IntakeDutyCycle);
    }

    public void extendBar() {
        barMotor.set(1d);
    }

    public void retractBar() {
        barMotor.set(-1d);
    }

    public void stopIntake() {
        intakeMotortop.stopMotor();
        intakeMotorbottom.stopMotor();
    }

    @Override
    public void periodic() {
        // TODO add logging

    }

}
