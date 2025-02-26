package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
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

    private final DigitalInput limitSwitchElevator;

    

    public static Elevator getInstance() {
        if (mInstance == null)
            mInstance = new Elevator();

        return mInstance;
    }

    public Runnable runElePID = () -> {

        elevatorMotorRight.setControl(new MotionMagicTorqueCurrentFOC(Constants.ElevatorConstants.kHomePosition));
        
    };

    public Elevator() {
        elevatorMotorRight = new TalonFX(Constants.ElevatorConstants.kRightElevatorMotorId);
        elevatorMotorLeft = new TalonFX(Constants.ElevatorConstants.kLeftElevatorMotorId);
        limitSwitchElevator = new DigitalInput(Constants.ElevatorConstants.kLimitSwitchId);

        elevatorMotionMagic = new MotionMagicTorqueCurrentFOC(0d);

        elevatorTargetMotorModel = DCMotor.getFalcon500(2);

        simElevatorTarget = new ElevatorSim(elevatorTargetMotorModel, Constants.ElevatorConstants.kElevatorGearing,
                Constants.ElevatorConstants.kElevatorWeight, Constants.ElevatorConstants.kDrumRadius,
                Constants.ElevatorConstants.kMinExtension, Constants.ElevatorConstants.kMaxExtension, false,
                0d);

        targetElevator_mechanism = new Mechanism2d(5, 5);
        targetElevatorBaseRoot = targetElevator_mechanism.getRoot("Target Elevator Root", 2.5, 0);
        m_targetElevatorMech2d = targetElevatorBaseRoot.append(
                new MechanismLigament2d("Elevator",
                        simElevatorTarget.getPositionMeters(), 90, 6, new Color8Bit(Color.kBlue)));

        simElevatorProfile = new ElevatorSim(elevatorTargetMotorModel, Constants.ElevatorConstants.kElevatorGearing,
                Constants.ElevatorConstants.kElevatorWeight, Constants.ElevatorConstants.kDrumRadius,
                Constants.ElevatorConstants.kMinExtension, Constants.ElevatorConstants.kMaxExtension, false,
                0d);

        profileElevator_mechanism = new Mechanism2d(20, 5);
        profileElevatorBaseRoot = profileElevator_mechanism.getRoot("Profile Elevator Root", 2.5, 0);
        m_profileElevatorMech2d = profileElevatorBaseRoot.append(
                new MechanismLigament2d("Elevator",
                        simElevatorProfile.getPositionMeters(), 90, 6, new Color8Bit(Color.kOrange)));

        TalonFXConfiguration elevatorMotorRightConfigs = new TalonFXConfiguration();

    
        elevatorMotorRightConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        elevatorMotorRightConfigs.Slot0.kS = Constants.ElevatorConstants.kElevatorS;
        elevatorMotorRightConfigs.Slot0.kG = Constants.ElevatorConstants.kElevatorG;

        elevatorMotorRightConfigs.Slot0.kV = Constants.ElevatorConstants.kElevatorV;
        elevatorMotorRightConfigs.Slot0.kA = Constants.ElevatorConstants.kElevatorA;

        elevatorMotorRightConfigs.Slot0.kP = Constants.ElevatorConstants.kElevatorP;
        elevatorMotorRightConfigs.Slot0.kI = Constants.ElevatorConstants.kElevatorI;
        elevatorMotorRightConfigs.Slot0.kD = Constants.ElevatorConstants.kElevatorD;

        elevatorMotorRightConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.ElevatorConstants.kElevatorCruiseVelocity;
        elevatorMotorRightConfigs.MotionMagic.MotionMagicAcceleration = Constants.ElevatorConstants.kElevatorAcceleration;

        // TODO do peak reverse output and current limits

        elevatorMotorRightConfigs.MotorOutput.PeakReverseDutyCycle =  Constants.ElevatorConstants.kElevatorPeakReverseDutyCycle;
        elevatorMotorRightConfigs.Feedback.SensorToMechanismRatio = Constants.ElevatorConstants.kRotationsToMeters;

        elevatorMotorRightConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        elevatorMotorRightConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ElevatorConstants.kMaxExtension;

        elevatorMotorRightConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        elevatorMotorRightConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ElevatorConstants.kMinExtension;

        elevatorMotorRightConfigs.CurrentLimits.StatorCurrentLimit = 60;

        /**
         * 
         */

        elevatorMotionMagic.Slot = 0;

        elevatorMotorRight.getConfigurator().apply(elevatorMotorRightConfigs);

        TalonFXConfiguration elevatorMotorLeftConfigs = new TalonFXConfiguration();

        elevatorMotorLeftConfigs.CurrentLimits.StatorCurrentLimit = 60;


        elevatorMotorLeft.getConfigurator().apply(elevatorMotorLeftConfigs);

        elevatorMotorLeft
                .setControl(new Follower(elevatorMotorRight.getDeviceID(), Constants.ElevatorConstants.kIsLeftInvert));

        SmartDashboard.putData("Elevator/Elevator Target", targetElevator_mechanism);
        SmartDashboard.putData("Elevator/Elevator Profile", profileElevator_mechanism);

    }

    public void setElevatorPositionMeters(double TargetElevatorMeters) {
        if (TargetElevatorMeters < Constants.ElevatorConstants.kMinExtension) {
            elevatorMotionMagic.Position = Constants.ElevatorConstants.kMinExtension;
        } else if (TargetElevatorMeters > Constants.ElevatorConstants.kMaxExtension) {
            elevatorMotionMagic.Position = Constants.ElevatorConstants.kMaxExtension;
        } else {
            elevatorMotionMagic.Position = TargetElevatorMeters;
        }
    }

    public Runnable zeroElevator = () -> {
        elevatorMotorRight.setPosition(0);
        postStatus("zeroed");
    };

    public void setElevatorDutyCycle(double elevatorDutyCycle) {
        elevatorMotorRight.set(elevatorDutyCycle);

        elevatorMotorLeft.setControl(new Follower(elevatorMotorRight.getDeviceID(), true));

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

    public boolean getLimitSwitch() {
        return limitSwitchElevator.get();
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Elevator/status", status);

    }

    public void updateSimElevatorTarget() {

        simElevatorTarget.setState(elevatorMotionMagic.Position, getElevatorVelocityMeters());

        simElevatorTarget.update(0.02);

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
        return elevatorMotorRight.getVelocity().getValueAsDouble();
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
            elevatorPositionMeters = elevatorMotorRight.getPosition().getValueAsDouble();
        }
    }

    public double getElevatorPositionMeters() {
        return elevatorPositionMeters;
    }

    public void setElevatorMotionMagic() {
        elevatorMotorRight.setControl(elevatorMotionMagic);
        elevatorMotorLeft
                .setControl(new Follower(elevatorMotorRight.getDeviceID(),Constants.ElevatorConstants.kIsLeftInvert));
    }


    @Override
    public void periodic() {

        if (Robot.isSimulation()) {
            updateSimElevatorTarget();
        } else {
            updateSimElevatorProfile();
        }

        updateElevatorPosition();

        SmartDashboard.putNumber("Elevator-Test/Torque current",
                elevatorMotorRight.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator-Test/Velocity", elevatorMotorRight.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Elevator-Test/Acceleration", elevatorMotorRight.getAcceleration().getValueAsDouble());

        // SmartDashboard.putNumber("Elevator-Test/Torque current over velocity -kg ",
        // elevatorPositionMeters);

        // SmartDashboard.putNumber("Elevator-Test/Torque current over acceleration -
        // kg", elevatorPositionMeters);

        SmartDashboard.putBoolean("Elevator-Test/Limit Switch ", limitSwitchElevator.get());

        SmartDashboard.putNumber("Elevator/Simulation/Position", simElevatorTarget.getPositionMeters());
        SmartDashboard.putNumber("Elevator/Velocity", getElevatorVelocityMeters());
        SmartDashboard.putNumber("Elevator/Target Elevator Meters", elevatorMotionMagic.Position);
        SmartDashboard.putNumber("Elevator/Position Meters", getElevatorPositionMeters());
        SmartDashboard.putBoolean("Elevator/At Position", isElevatorAtTarget());

        SmartDashboard.putNumber("Elevator/elevator position", elevatorMotorRight.getPosition().getValueAsDouble());

        SmartDashboard.putBoolean("Elevator/brownout right", elevatorMotorRight.getFault_BridgeBrownout().getValue());
        SmartDashboard.putBoolean("Elevator/brownout left", elevatorMotorLeft.getFault_BridgeBrownout().getValue());

        // SmartDashboard.putNumber("right stator current limit",
        //         elevatorMotorRight.getStatorCurrent().getValueAsDouble());

    }
}
