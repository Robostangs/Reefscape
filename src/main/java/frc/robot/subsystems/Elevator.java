package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {

    private static Elevator mInstance;

    // real elevator
    private TalonFX elevatorMotorRight;
    private TalonFX elevatorMotorLeft;

    private CANcoder elevatorEncoderRight;
    private CANcoder elevatorEncoderLeft;

    private MotionMagicTorqueCurrentFOC elevatorMotionMagic;
    private double elevatorPositionMeters;

    // simulated elevator
    private ElevatorSim simElevatorTarget;
    private DCMotor elevatorTargetMotorModel;

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d targetElevator_mechanism;
    private final MechanismRoot2d targetElevatorBaseRoot;
    private final MechanismLigament2d m_targetElevatorMech2d;

    // simulated elevator
    private ElevatorSim simElevatorProfile;

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d profileElevator_mechanism;
    private final MechanismRoot2d profileElevatorBaseRoot;
    private final MechanismLigament2d m_profileElevatorMech2d;

    

    public static Elevator getInstance() {
        if (mInstance == null)
            mInstance = new Elevator();

        return mInstance;
    }

    public Elevator() {
        elevatorMotorRight = new TalonFX(Constants.ElevatorConstants.kRightElevatorMotorId);
        elevatorEncoderRight = new CANcoder(Constants.ElevatorConstants.kRightElevatorEncoderId);
        elevatorMotorLeft = new TalonFX(Constants.ElevatorConstants.kLeftElevatorMotorId);
        elevatorEncoderLeft = new CANcoder(Constants.ElevatorConstants.kLeftElevatorEncoderId);

        elevatorMotionMagic = new MotionMagicTorqueCurrentFOC(0d)
                .withFeedForward(Constants.ElevatorConstants.kElevatorFF);

        elevatorTargetMotorModel = DCMotor.getFalcon500(1);

        simElevatorTarget = new ElevatorSim(elevatorTargetMotorModel, Constants.ElevatorConstants.kElevatorGearing,
                Constants.ElevatorConstants.kElevatorWeight, Constants.ElevatorConstants.kDrumRadius,
                Constants.ElevatorConstants.kMinElevatorHeight, Constants.ElevatorConstants.kMaxElevatorHeight, false,
                0d);

        targetElevator_mechanism = new Mechanism2d(20, 50);
        targetElevatorBaseRoot = targetElevator_mechanism.getRoot("Target Elevator Root", 10, 0);
        m_targetElevatorMech2d = targetElevatorBaseRoot.append(
                new MechanismLigament2d("Elevator",
                        simElevatorTarget.getPositionMeters(), 90, 6, new Color8Bit(Color.kBlue)));

        simElevatorProfile = new ElevatorSim(elevatorTargetMotorModel, Constants.ElevatorConstants.kElevatorGearing,
                Constants.ElevatorConstants.kElevatorWeight, Constants.ElevatorConstants.kDrumRadius,
                Constants.ElevatorConstants.kMinElevatorHeight, Constants.ElevatorConstants.kMaxElevatorHeight, false,
                0d);

        profileElevator_mechanism = new Mechanism2d(20, 50);
        profileElevatorBaseRoot = profileElevator_mechanism.getRoot("Profile Elevator Root", 10, 0);
        m_profileElevatorMech2d = profileElevatorBaseRoot.append(
                new MechanismLigament2d("Elevator",
                        simElevatorProfile.getPositionMeters(), 90, 6, new Color8Bit(Color.kOrange)));

        var slot0Configs = new Slot0Configs();
        // TODO tune these values
        slot0Configs.kP = Constants.ElevatorConstants.kElevatorP;
        slot0Configs.kI = Constants.ElevatorConstants.kElevatorI;
        slot0Configs.kD = Constants.ElevatorConstants.kElevatorD;
        slot0Configs.kS = Constants.ElevatorConstants.kElevatorFF;
        TalonFXConfiguration elevatorMotorRightConfigs = new TalonFXConfiguration();
        elevatorMotorRightConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        elevatorMotorRightConfigs.Feedback.RotorToSensorRatio = 100;
        elevatorMotorRightConfigs.Feedback.SensorToMechanismRatio = Constants.ElevatorConstants.kRotationsToMeters;
        elevatorMotorRightConfigs.Feedback.FeedbackRemoteSensorID = elevatorEncoderRight.getDeviceID();
        elevatorMotionMagic.Slot = 0;

        elevatorMotorRight.getConfigurator().apply(elevatorMotorRightConfigs);
        elevatorMotorRight.getConfigurator().apply(slot0Configs);


        SmartDashboard.putData("Elevator/Elevator Sim", targetElevator_mechanism);
        SmartDashboard.putData("Elevator/Elevator Profile", profileElevator_mechanism);

    }

    public void setElevatorPositionMeters(double TargetElevatorMeters) {
        if (TargetElevatorMeters < Constants.ElevatorConstants.kMinElevatorHeight) {
            elevatorMotionMagic.Position = Constants.ElevatorConstants.kMinElevatorHeight;
        } else if (TargetElevatorMeters > Constants.ElevatorConstants.kMaxElevatorHeight) {
            elevatorMotionMagic.Position = Constants.ElevatorConstants.kMaxElevatorHeight;
        } else {
            elevatorMotionMagic.Position = TargetElevatorMeters;
        }
    }

    public void setElevatorDutyCycle(double elevatorDutyCycle) {
        elevatorMotorRight.set(elevatorDutyCycle);
        elevatorMotorLeft.set(elevatorDutyCycle);
        
    }

    public void setElevatorPosition(double elevatorPosition) {
        elevatorMotorRight.setPosition(elevatorPosition);
        elevatorMotorLeft.setPosition(elevatorPosition);
    }

    public void setStatorCurrentLimit(double currentLimit) {
        TalonFXConfiguration elevatorCurrentConfigs = new TalonFXConfiguration();
        elevatorCurrentConfigs.CurrentLimits.StatorCurrentLimit = currentLimit;
        elevatorMotorRight.getConfigurator().apply(elevatorCurrentConfigs);
        elevatorMotorLeft.getConfigurator().apply(elevatorCurrentConfigs);

    }

    
    public void postStatus(String status) {
        SmartDashboard.putString("Elevator/status", status);

    }

    public void updateSimElevatorTarget() {

        simElevatorTarget.setState(elevatorMotionMagic.Position, getElevatorVelocityMeters());

        simElevatorTarget.update(0.02);

        elevatorEncoderRight.getSimState()
                .setRawPosition(simElevatorTarget.getPositionMeters() / Constants.ElevatorConstants.kRotationsToMeters);

        m_targetElevatorMech2d.setLength(simElevatorTarget.getPositionMeters());

    }

    public void updateSimElevatorProfile() {

        // Set the input to the simulated elevator using the torque current from the
        // motor
        simElevatorProfile.setInput(elevatorMotorRight.getTorqueCurrent().getValueAsDouble());

        simElevatorProfile.update(0.02);

        m_profileElevatorMech2d.setLength(
                simElevatorTarget.getPositionMeters());

    }

    public double getElevatorVelocityMeters() {
        return elevatorMotorRight.getVelocity().getValueAsDouble() / Constants.ElevatorConstants.kRotationsToMeters;
    }

    public boolean isElevatorAtTarget() {
        if (Math.abs(elevatorPositionMeters - elevatorMotionMagic.Position) < 0.01) {
            return true;
        } else {
            return false;
        }
    }

    public void updateElevatorPosition() {
        if (Robot.isSimulation()) {
            elevatorPositionMeters = simElevatorTarget.getPositionMeters();
        } else {
            elevatorPositionMeters = Constants.ElevatorConstants.kRotationsToMeters
                    * elevatorMotorRight.getPosition().getValueAsDouble();
        }
    }

    public double getElevatorPositionMeters() {
        return elevatorPositionMeters;
    }

    public void setElevatorMotionMagic(){
        elevatorMotorRight.setControl(elevatorMotionMagic);
        elevatorMotorLeft.setControl(new Follower(Constants.ElevatorConstants.kRightElevatorMotorId, Constants.ElevatorConstants.kIsLeftInvert));
    }

    @Override
    public void periodic() {



        if (Robot.isSimulation()) {
            updateSimElevatorTarget();
        } else {
            updateSimElevatorProfile();
        }

        updateElevatorPosition();

        SmartDashboard.putNumber("Elevator/Simulation/Position", simElevatorTarget.getPositionMeters());
        SmartDashboard.putNumber("Elevator/Real/Encoder Position", elevatorEncoderRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Real/Velocity", getElevatorVelocityMeters());
        SmartDashboard.putNumber("Elevator/Real/Target Elevator Meters", elevatorMotionMagic.Position);
        SmartDashboard.putNumber("Elevator/Real/Position Meters", getElevatorPositionMeters());
        SmartDashboard.putBoolean("Elevator/Real/At Position", isElevatorAtTarget());

    }
}
