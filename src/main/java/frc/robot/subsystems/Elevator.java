package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    public Elevator() {
        elevatorMotorRight = new TalonFX(Constants.ElevatorConstants.kRightElevatorMotorId);
        elevatorMotorLeft = new TalonFX(Constants.ElevatorConstants.kLeftElevatorMotorId);
        limitSwitchElevator = new DigitalInput(Constants.ElevatorConstants.kLimitSwitchId);


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

        // TODO tune these values
        TalonFXConfiguration elevatorMotorRightConfigs = new TalonFXConfiguration();

        /**
         * double check limlit switvh
         * put in softlimits
         * find kg
         * find kv
         * find ka
         * fina cruise velocity
         * 
         * 
         * torquqe current = kg +kv*v + ka*a
         * 
         */

        elevatorMotorRightConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        elevatorMotorRightConfigs.Slot0.kG = Constants.ElevatorConstants.kElevatorG;

        // elevatorMotorRightConfigs.Slot0.StaticFeedforwardSign =
        // StaticFeedforwardSignValue.UseVelocitySign;

        // elevatorMotorRightConfigs.Slot0.kS = Constants.ElevatorConstants.kElevatorFF;

        elevatorMotorRightConfigs.Slot0.kP = Constants.ElevatorConstants.kElevatorP;
        elevatorMotorRightConfigs.Slot0.kI = Constants.ElevatorConstants.kElevatorI;
        elevatorMotorRightConfigs.Slot0.kD = Constants.ElevatorConstants.kElevatorD;

        elevatorMotorRightConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.ElevatorConstants.kElevatorCruiseVelocity;

        // Dividing the supply voltage by kA results in the maximum acceleration of the
        // profile from 0.
        elevatorMotorRightConfigs.MotionMagic.MotionMagicExpo_kA = Constants.ElevatorConstants.kElevatorA;

        // Dividing the supply voltage by kV results in the maximum velocity of the
        // profile.
        elevatorMotorRightConfigs.MotionMagic.MotionMagicExpo_kV = Constants.ElevatorConstants.kElevatorV;

        elevatorMotorRightConfigs.Feedback.SensorToMechanismRatio = Constants.ElevatorConstants.kRotationsToMeters;

        // elevatorMotorRightConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // elevatorMotorRightConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        elevatorMotorRightConfigs.CurrentLimits.StatorCurrentLimit = 30;






        /**
         * 
         */




        elevatorMotionMagic.Slot = 0;

        elevatorMotorRight.getConfigurator().apply(elevatorMotorRightConfigs);

        TalonFXConfiguration elevatorMotorLeftConfigs = new TalonFXConfiguration();

        elevatorMotorLeftConfigs.CurrentLimits.StatorCurrentLimit = 30;



        elevatorMotorLeft.getConfigurator().apply(elevatorMotorLeftConfigs);



        // elevatorMotorLeft.follow(elevatorMotorRight);


        elevatorMotorLeft
                .setControl(new Follower(elevatorMotorRight.getDeviceID(), Constants.ElevatorConstants.kIsLeftInvert));

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



    public  Runnable zeroElevator = () -> {
        elevatorMotorRight.setPosition(0);
        postStatus("zeroed");
    };
    public void setElevatorDutyCycle(double elevatorDutyCycle) {
        elevatorMotorRight.set(elevatorDutyCycle);
        
        elevatorMotorLeft.set(-elevatorDutyCycle);

    }

    public void setBrakeMode() {
        elevatorMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotorRight.setNeutralMode(NeutralModeValue.Brake);

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

    public boolean getLimitSwitch(){
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

    public void setElevatorMotionMagic() {
        elevatorMotorRight.setControl(elevatorMotionMagic);
        // elevatorMotorLeft
        // .setControl(new Follower(elevatorMotorRight.getDeviceID(),
        // Constants.ElevatorConstants.kIsLeftInvert));
    }

    /**
     * * torquqe current = kg +kv*v + ka*a
     * 
     */

    @Override
    public void periodic() {

        if (Robot.isSimulation()) {
            updateSimElevatorTarget();
        } else {
            updateSimElevatorProfile();
        }

        updateElevatorPosition();

        SmartDashboard.putNumber("Elevator-Test/Torque current", elevatorMotorRight.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator-Test/Veleocity", elevatorMotorRight.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Elevator-Test/Acceleration", elevatorMotorRight.getAcceleration().getValueAsDouble());

        // SmartDashboard.putNumber("Elevator-Test/Torque current over velocity -kg ", elevatorPositionMeters);

        // SmartDashboard.putNumber("Elevator-Test/Torque current over acceleration - kg", elevatorPositionMeters);

        SmartDashboard.putBoolean("Elevator-Test/Limit Switch ", limitSwitchElevator.get());

        SmartDashboard.putNumber("Elevator/Simulation/Position", simElevatorTarget.getPositionMeters());
        SmartDashboard.putNumber("Elevator/Real/Velocity", getElevatorVelocityMeters());
        SmartDashboard.putNumber("Elevator/Real/Target Elevator Meters", elevatorMotionMagic.Position);
        SmartDashboard.putNumber("Elevator/Real/Position Meters", getElevatorPositionMeters());
        SmartDashboard.putBoolean("Elevator/Real/At Position", isElevatorAtTarget());

        SmartDashboard.putNumber("Elevator/elevator position", elevatorMotorRight.getPosition().getValueAsDouble());

        SmartDashboard.putBoolean("Elevator/brownout right", elevatorMotorRight.getFault_BridgeBrownout().getValue());
        SmartDashboard.putBoolean("Elevator/brownout left", elevatorMotorLeft.getFault_BridgeBrownout().getValue());

SmartDashboard.putNumber("right stator current limit", elevatorMotorRight.getStatorCurrent().getValueAsDouble());


    }
}
