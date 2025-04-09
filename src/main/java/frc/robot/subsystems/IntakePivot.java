package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
    private TalonFX pivotMotor;
    private static IntakePivot mInstance;
    MotionMagicTorqueCurrentFOC pivotControl;


    public static IntakePivot getInstance() {
        if (mInstance == null)
            mInstance = new IntakePivot();
        return mInstance;
    }

    public IntakePivot() {

        pivotMotor = new TalonFX(Constants.IntakeConstants.kPivotMotorId);


        TalonFXConfiguration pivotMotorConfigs = new TalonFXConfiguration();

        pivotMotorConfigs.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.kStatorCurrentLimit;

        pivotMotorConfigs.Feedback.SensorToMechanismRatio =  Constants.IntakeConstants.kSensorToMechanismRatio;

        pivotMotorConfigs.Slot0.kP = Constants.IntakeConstants.kPivotP;
        pivotMotorConfigs.Slot0.kI = Constants.IntakeConstants.kPivotI;
        pivotMotorConfigs.Slot0.kD = Constants.IntakeConstants.kPivotD;
        pivotMotorConfigs.Slot0.kG = Constants.IntakeConstants.kPivotG;
        pivotMotorConfigs.Slot0.kS = Constants.IntakeConstants.kPivotS;
        pivotMotorConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakeConstants.kMotionMagicVelocity;
        pivotMotorConfigs.MotionMagic.MotionMagicAcceleration = Constants.IntakeConstants.kMotionMagicAcceleration;


        
        pivotMotor.getConfigurator().apply(pivotMotorConfigs);

        pivotControl = new MotionMagicTorqueCurrentFOC(pivotMotor.getPosition().getValueAsDouble());

    }

    public double getIntakePosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public Runnable zeroIntakeRun = () -> {
        point3Intake();
    };

    
    public void point3Intake() {
        pivotMotor.setPosition(Constants.IntakeConstants.kHardstopPosition);
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

    public void setHeimlichPosition() {
        pivotControl.Position = Constants.IntakeConstants.kHeimlichSetpoint;
    }
    public void setAlgaeIntakePosition(){
        pivotControl.Position = Constants.IntakeConstants.kAlgaeInSetpoint;
    }
    public void setAlgaeOutPosition(){
        pivotControl.Position = Constants.IntakeConstants.kAlgaeOutSetpoint;
    }
    public void postStatus(String status) {
        SmartDashboard.putString("Intake/status", status);

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

    public void setPiviotDutyCycle(double pivotDutyCycle) {
        pivotMotor.set(pivotDutyCycle);
    }


    @Override
    public void periodic() {
        pivotMotor.setControl(pivotControl);

        // Robot.verifyMotor(pivotMotor);

        SmartDashboard.putNumber("Intake/Setpoint", pivotControl.Position);
        SmartDashboard.putNumber("Intake/Position", pivotMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putBoolean("Intake/is at extend setpoint", isIntakeatSetpoint(true));
        // SmartDashboard.putBoolean("Intake/is at retract setpoint", isIntakeatSetpoint(false));
        // SmartDashboard.putNumber("Intake/Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());

    }

}
