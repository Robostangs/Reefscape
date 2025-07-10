package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private static Climber mInstance;
    private TalonFX climberMotor;
    private Servo climbServo;

    public static Climber getInstance() {
        if (mInstance == null)
            mInstance = new Climber();
        return mInstance;
    }

    public Climber() {
        climbServo = new Servo(Constants.ClimberConstants.kServoId);
        climberMotor = new TalonFX(Constants.ClimberConstants.kClimberMotorId);

        TalonFXConfiguration climberMotorConfigs = new TalonFXConfiguration();
        climberMotorConfigs.Feedback.SensorToMechanismRatio = Constants.ClimberConstants.kGearboxRotationsToMechanismMeters;
        climberMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        climberMotor.getConfigurator().apply(climberMotorConfigs);

    }

    public double getServoPosition() {
        return climbServo.getAngle();
    }

    public void runClimber(double climberDutyCycle) {
        climberMotor.set(climberDutyCycle);
    }

    public void setServoAngle(double servoAngle) {
        climbServo.setAngle(servoAngle);
    }

    public Runnable zeroClimberPosition = () -> {
        climberMotor.setPosition(0);
    };

    public void zeroClimber() {
        climberMotor.setPosition(0);

    }

    public void postStatus(String status) {
        SmartDashboard.putString("Climber/status", status);

    }

    public double getClimberPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Climber/Kraken Position", getClimberPosition());
    }

}
