package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private static Climber mInstance;
    private TalonFX climberMotor;
    public static Climber getInstance() {
        if (mInstance == null)
            mInstance = new Climber();
        return mInstance;
    }

    public Climber() {
        climberMotor = new TalonFX(Constants.ClimberConstants.kClimberMotorId);


        TalonFXConfiguration climberMotorConfigs = new TalonFXConfiguration();
        climberMotorConfigs.Feedback.SensorToMechanismRatio = Constants.ClimberConstants.kGearboxRotationsToMechanismMeters;

        climberMotor.getConfigurator().apply(climberMotorConfigs);


    }

    public void runClimber(double climberDutyCycle) {
        climberMotor.set(climberDutyCycle);
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Climber/status", status);

    }

    public void setBrake() {
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public double getClimberPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }
  
    @Override
    public void periodic() {

        climberMotor.getPosition().getValueAsDouble();
    }
    
}
