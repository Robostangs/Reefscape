package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climber extends SubsystemBase {
    private static Climber mInstance;
    private TalonFX climberMotor;
    private Servo cliServo;

    //
    public static Climber getInstance() {
        if (mInstance == null)
            mInstance = new Climber();
        return mInstance;
    }

    public Climber() {
        cliServo = new Servo(Constants.ClimberConstants.kServoId);
        climberMotor = new TalonFX(Constants.ClimberConstants.kClimberMotorId);

        TalonFXConfiguration climberMotorConfigs = new TalonFXConfiguration();
        climberMotorConfigs.Feedback.SensorToMechanismRatio = Constants.ClimberConstants.kGearboxRotationsToMechanismMeters;
        climberMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        climberMotor.getConfigurator().apply(climberMotorConfigs);

    }

    public double getServoPosition() {
        return cliServo.getAngle();
    }

    public void runClimber(double climberDutyCycle) {
        climberMotor.set(climberDutyCycle);
    }

    public void setServoAngle(double servoDutyCycle) {
        cliServo.setAngle(servoDutyCycle);

    }

    public Runnable zeroClimberPosition = () -> {
        climberMotor.setPosition(0);
    };

    public Runnable zeroServo = () -> {
        cliServo.setAngle(0);
    };

    public Runnable goToservpos = () -> {
        cliServo.setAngle(107);
    };

    public void zeroClimber() {
        climberMotor.setPosition(0);

    }

    public void postStatus(String status) {
        SmartDashboard.putString("Climber/status", status);

    }

    public void setBrake() {
        // climberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public double getClimberPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {

        // Robot.verifyMotor(climberMotor);

        SmartDashboard.putNumber("Climber/Kraken Position", climberMotor.getPosition().getValueAsDouble());
    }

}
