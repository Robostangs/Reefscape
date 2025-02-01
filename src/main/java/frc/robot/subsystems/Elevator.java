package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * 1. sim elevator rename to sim elevator target
 *   </p>

 * 2. target elevator sim -> sim elevator profile- updatesimelevatorprofile()
 * </p>
 * 3.in periodic before update elevator postion runs: above update elevator
 * position- run update sim elevator target
 * if(robo real){update the sim elevator profile} - color different
 *  * </p>

 * 4. build one big mechanism for all
 * 
 */
public class Elevator extends SubsystemBase {

    private static Elevator mInstance;

    // real elevator
    private TalonFX elevatorMotor;
    private CANcoder elevatorEncoder;

    private MotionMagicTorqueCurrentFOC elevatorMotionMagic;
    private double elevatorPositionMeters;

    // simulated elevator
    private ElevatorSim simElevatorTarget;
    private DCMotor elevatorTargetMotorModel;

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d targetElevator_mechanism;
    private final MechanismRoot2d targetElevatorBaseRoot;
    private final MechanismLigament2d m_targetElevatorMech2d;

    public static Elevator getInstance() {
        if (mInstance == null)
            mInstance = new Elevator();

        return mInstance;
    }

    public Elevator() {
        elevatorMotor = new TalonFX(Constants.ElevatorConstants.kElevatorMotorId);
        elevatorEncoder = new CANcoder(Constants.ElevatorConstants.kElevatorEncoderId);

        elevatorMotionMagic = new MotionMagicTorqueCurrentFOC(0d)
                .withFeedForward(Constants.ElevatorConstants.kElevatorFF);

        elevatorTargetMotorModel = DCMotor.getFalcon500(1);

        simElevatorTarget = new ElevatorSim(elevatorTargetMotorModel, Constants.ElevatorConstants.kElevatorGearing,
                Constants.ElevatorConstants.kElevatorWeight, Constants.ElevatorConstants.kDrumRadius,
                Constants.ElevatorConstants.kMinElevatorHeight, Constants.ElevatorConstants.kMaxElevatorHeight, true,
                0d);

        targetElevator_mechanism = new Mechanism2d(20, 50);
        targetElevatorBaseRoot = targetElevator_mechanism.getRoot("Elevator Root", 10, 0);
        m_targetElevatorMech2d = targetElevatorBaseRoot.append(
                new MechanismLigament2d("Elevator",
                        simElevatorTarget.getPositionMeters(), 90));

        var slot0Configs = new Slot0Configs();
        // TODO tune these values
        slot0Configs.kP = Constants.ElevatorConstants.kElevatorP;
        slot0Configs.kI = Constants.ElevatorConstants.kElevatorI;
        slot0Configs.kD = Constants.ElevatorConstants.kElevatorD;
        slot0Configs.kS = Constants.ElevatorConstants.kElevatorFF;
        TalonFXConfiguration elevatorMotorConfigs = new TalonFXConfiguration();
        elevatorMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        elevatorMotorConfigs.Feedback.RotorToSensorRatio = 100;
        elevatorMotorConfigs.Feedback.SensorToMechanismRatio = 1;
        elevatorMotorConfigs.Feedback.FeedbackRemoteSensorID = elevatorEncoder.getDeviceID();
        elevatorMotionMagic.Slot = 0;

        elevatorMotor.getConfigurator().apply(slot0Configs);

        SmartDashboard.putData("Elevator/Elevator Sim", targetElevator_mechanism);

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

    public void postStatus(String status) {
        SmartDashboard.putString("Elevator/status", status);

    }

    public void updateSimElevatorTarget() {

        simElevatorTarget.setInput(elevatorMotor.getSimState().getTorqueCurrent());

        simElevatorTarget.update(0.02);

        elevatorEncoder.getSimState()
                .setRawPosition(simElevatorTarget.getPositionMeters() / Constants.ElevatorConstants.kRotationsToMeters);

        m_targetElevatorMech2d.setLength(simElevatorTarget.getPositionMeters());

    }

    public void updateSimElevatorProfile() {

        // Set the input to the simulated elevator using the torque current from the
        // motor
        simElevatorTarget.setInput(elevatorMotor.getTorqueCurrent().getValueAsDouble());

        simElevatorTarget.update(0.02);

        m_targetElevatorMech2d.setLength(
                elevatorMotor.getPosition().getValueAsDouble() * Constants.ElevatorConstants.kRotationsToMeters);

    }

    public double getElevatorVelocityMeters() {
        return elevatorMotor.getVelocity().getValueAsDouble() / Constants.ElevatorConstants.kRotationsToMeters;
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
                    * elevatorMotor.getPosition().getValueAsDouble();
        }
    }

    public double getElevatorPositionMeters() {
        return elevatorPositionMeters;
    }

    @Override
    public void periodic() {

        if (Robot.isSimulation()) {
            updateSimElevatorTarget();
        }


        updateElevatorPosition();

        elevatorMotor.setControl(elevatorMotionMagic
                .withPosition(elevatorMotionMagic.Position * Constants.ElevatorConstants.kRotationsToMeters));

   
        

        SmartDashboard.putNumber("Elevator/Simulation/Position", simElevatorTarget.getPositionMeters());
        SmartDashboard.putNumber("Elevator/Real/Encoder Position", elevatorEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Real/Velocity", getElevatorVelocityMeters());
        SmartDashboard.putNumber("Elevator/Real/Target Elevator Meters", elevatorMotionMagic.Position);
        SmartDashboard.putNumber("Elevator/Real/Position Meters", getElevatorPositionMeters());
        SmartDashboard.putBoolean("Elevator/Real/At Position", isElevatorAtTarget());

    }
}
