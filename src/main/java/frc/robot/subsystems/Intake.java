package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotorTop, piviotMotor;
    private Alert intakeAlert = new Alert("INTAKE TWEAKING", Alert.AlertType.kError);
    private static Intake mInstance;
    private DigitalInput IntakeSensor;
    MotionMagicExpoTorqueCurrentFOC piviotControl;

    public static Intake getInstance() {
        if (mInstance == null)
            mInstance = new Intake();
        return mInstance;
    }

    public Intake() {

        intakeMotorTop = new TalonFX(Constants.IntakeConstants.kIntakeMotorId);
        piviotMotor = new TalonFX(Constants.IntakeConstants.kBarMotorId);
        IntakeSensor = new DigitalInput(Constants.IntakeConstants.kIntakeSensorId);

        var slotpiviotconfigs = new Slot0Configs();
        slotpiviotconfigs.kP = Constants.IntakeConstants.kPiviotP;
        slotpiviotconfigs.kI = Constants.IntakeConstants.kPiviotI;
        slotpiviotconfigs.kD = Constants.IntakeConstants.kPiviotD;
        slotpiviotconfigs.kS = Constants.IntakeConstants.kPiviots;

        TalonFXConfiguration piviotMotorConfigs = new TalonFXConfiguration();
        piviotMotorConfigs.Feedback.SensorToMechanismRatio = 1.0;
        piviotMotorConfigs.CurrentLimits.SupplyCurrentLimit = 30;
        

        piviotMotor.getConfigurator().apply(piviotMotorConfigs);
        piviotMotor.getConfigurator().apply(slotpiviotconfigs);


        piviotControl = new MotionMagicExpoTorqueCurrentFOC(0d);

        
    }

    public void runIntake(double IntakeDutyCycle) {
        intakeMotorTop.set(IntakeDutyCycle);

    }

    public void extendBar() {
        piviotControl.Position = Constants.IntakeConstants.kExtendSetpoint;
    }

    public void stopBar() {
        piviotMotor.set(0);
    }

    public void retractBar() {
        piviotControl.Position = Constants.IntakeConstants.kRetractSetpoint;

    }

    public void postStatus(String status) {
        SmartDashboard.putString("Intake/status", status);

    }

    public void stopIntake() {
        intakeMotorTop.stopMotor();
        piviotControl.Position = piviotMotor.getPosition().getValueAsDouble();
    }

    public boolean getIntakeSensor() {
        return IntakeSensor.get();
    }

    public void setIntakePiviotBrake() {
        piviotMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    public boolean isIntakeatSetpoint(boolean extendorretract) {

        if (extendorretract) {
            return piviotMotor.getPosition().getValueAsDouble() > Constants.IntakeConstants.kExtendSetpoint+0.1;
        } else {
            return piviotMotor.getPosition().getValueAsDouble() < Constants.IntakeConstants.kExtendSetpoint-0.1;

        }
    }

    @Override
    public void periodic() {
        // TODO add logging
        SmartDashboard.putNumber("Intake/Setpoint", piviotControl.Position);
        SmartDashboard.putNumber("Intake/Position", piviotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("is at extend setpoint", isIntakeatSetpoint(true));
        SmartDashboard.putBoolean("is at retract setpoint", isIntakeatSetpoint(false));


        piviotMotor.setControl(piviotControl);

    }

}
