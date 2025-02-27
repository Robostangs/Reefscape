package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
    private TalonFX pivotMotor;
    private Alert intakeAlert = new Alert("INTAKE TWEAKING", Alert.AlertType.kError);
    private static IntakePivot mInstance;
    MotionMagicExpoTorqueCurrentFOC pivotControl;

    public static IntakePivot getInstance() {
        if (mInstance == null)
            mInstance = new IntakePivot();
        return mInstance;
    }

    public IntakePivot() {

        pivotMotor = new TalonFX(Constants.IntakeConstants.kPivotMotorId);

        var slotpivotconfigs = new Slot0Configs();
        slotpivotconfigs.kP = Constants.IntakeConstants.kPivotP;
        slotpivotconfigs.kI = Constants.IntakeConstants.kPivotI;
        slotpivotconfigs.kD = Constants.IntakeConstants.kPivotD;
        slotpivotconfigs.kS = Constants.IntakeConstants.kPivotS;

        TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration();
        pivotMotorConfigs.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.kStatorCurrentLimit;

        pivotMotor.getConfigurator().apply(pivotMotorConfigs);
        pivotMotor.getConfigurator().apply(slotpivotconfigs);

        pivotControl = new MotionMagicExpoTorqueCurrentFOC(pivotMotor.getPosition().getValueAsDouble());

    }

    public double getIntakePosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public Runnable zeroIntakeRun = () -> {
        pivotMotor.setPosition(0);
    };

    public void zeroIntake() {
        pivotMotor.setPosition(0d);
    }

    public void setExtendPosition() {
        pivotControl.Position = Constants.IntakeConstants.kExtendSetpoint;
    }

    public void stopBar() {
        pivotMotor.set(0);
    }

    public void setRetractPosition() {
        pivotControl.Position = Constants.IntakeConstants.kRetractSetpoint;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Intake/status", status);

    }

    public void setPivotZero() {
        pivotMotor.setPosition(0);
    }

    public void stopIntake() {
        pivotControl.Position = pivotMotor.getPosition().getValueAsDouble();
    }

    public void setIntakePivotBrake() {
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    public void setStatorCurrentLimit(double currentLimit) {
        TalonFXConfiguration intakeCurrentConfigs = new TalonFXConfiguration();
        intakeCurrentConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;
        pivotMotor.getConfigurator().apply(intakeCurrentConfigs);

    }

    public boolean isIntakeatSetpoint(boolean extendorretract) {

        if (extendorretract) {
            return pivotMotor.getPosition().getValueAsDouble() <= Constants.IntakeConstants.kExtendSetpoint + 4;
        } else {
            return pivotMotor.getPosition().getValueAsDouble() >= Constants.IntakeConstants.kRetractSetpoint - 0.5;

        }
    }

    // public void runIntakeMotionMagic() {
    // pivotMotor.setControl(pivotControl);

    // }
    public void setPiviotDutyCycle(double pivotDutyCycle) {
        pivotMotor.set(pivotDutyCycle);
    }

    @Override
    public void periodic() {
        pivotMotor.setControl(pivotControl);
        // TODO add logging
        SmartDashboard.putNumber("Intake/Setpoint", pivotControl.Position);
        SmartDashboard.putNumber("Intake/Position", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/is at extend setpoint", isIntakeatSetpoint(true));
        SmartDashboard.putBoolean("Intake/is at retract setpoint", isIntakeatSetpoint(false));
        SmartDashboard.putNumber("Intake/Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());

    }

}
