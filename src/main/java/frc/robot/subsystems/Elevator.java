package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {

    private static Elevator mInstance;

    // real elevator
    private TalonFX elevatorMotor;
    private CANcoder elevatorEncoder;
    private double TargetElevatorRotations = 0d;

    private boolean isElevatorAtTarget = false;
    private MotionMagicTorqueCurrentFOC elevatorMotionMagic;

    // simulated elevator
    private ElevatorSim simElevator;
    private DCMotor elevatorMotorModel;

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d elevator_mechanism;
    private final MechanismRoot2d elevatorBaseRoot;
    private final MechanismLigament2d m_elevatorMech2d;

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

        elevatorMotorModel = DCMotor.getFalcon500(1);

        simElevator = new ElevatorSim(elevatorMotorModel, Constants.ElevatorConstants.kElevatorGearing,
                Constants.ElevatorConstants.kElevatorWeight, Constants.ElevatorConstants.kDrumRadius,
                Constants.ElevatorConstants.kMinElevatorHeight, Constants.ElevatorConstants.kmaxElevatorHeight, true,
                0d);

        elevator_mechanism = new Mechanism2d(20, 50);
        elevatorBaseRoot = elevator_mechanism.getRoot("Elevator Root", 10, 0);
        m_elevatorMech2d = elevatorBaseRoot.append(
                new MechanismLigament2d("Elevator",
                        simElevator.getPositionMeters(), 90));

        var slot0Configs = new Slot0Configs();
        // TODO tune these values
        slot0Configs.kP = Constants.ElevatorConstants.kElevatorP;
        slot0Configs.kI = Constants.ElevatorConstants.kElevatorI;
        slot0Configs.kD = Constants.ElevatorConstants.kElevatorD;
        slot0Configs.kS = Constants.ElevatorConstants.kElevatorFF;

        elevatorMotor.getConfigurator().apply(slot0Configs);

        SmartDashboard.putData("Elevator/Elevator Sim", elevator_mechanism);

    }

    public void setElevatorPosition(double TargetElevatorMeters) {
        if (TargetElevatorMeters > Constants.ElevatorConstants.kMinElevatorHeight) {
            this.TargetElevatorRotations = Constants.ElevatorConstants.kMinElevatorHeight;
        } else if (TargetElevatorMeters < Constants.ElevatorConstants.kmaxElevatorHeight) {
            this.TargetElevatorRotations = Constants.ElevatorConstants.kmaxElevatorHeight;
        } else {
            this.TargetElevatorRotations = TargetElevatorMeters;
        }

    }

    public void postStatus(String status) {
        SmartDashboard.putString("Elevator/status", status);

    }

    public void updateSimElevator() {

        // Set the input to the simulated elevator using the torque current from the
        // motor
        simElevator.setInput(elevatorMotor.getSimState().getTorqueCurrent());

        simElevator.update(0.02);

        elevatorEncoder.getSimState()
                .setRawPosition(simElevator.getPositionMeters() / Constants.ElevatorConstants.kRotationstoMeters);

        m_elevatorMech2d.setLength(simElevator.getPositionMeters());

    }

    public boolean getIsElevatorAtTarget() {
        return isElevatorAtTarget;
    }

    // make functions for the smart dashboard variables
    // make folders in smart dashboard for the variables(real and simulated)

    @Override
    public void periodic() {

        elevatorMotor.setControl(elevatorMotionMagic.withPosition(TargetElevatorRotations));

        if (Robot.isSimulation()) {
            updateSimElevator();
        }

        SmartDashboard.putNumber("Elevator/Simulation Position", simElevator.getPositionMeters());
        SmartDashboard.putNumber("Elevator/Encoder Position", elevatorEncoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Velocity", simElevator.getVelocityMetersPerSecond());
        SmartDashboard.putNumber("Elevator/TargetElevatorMeters", TargetElevatorRotations);
        SmartDashboard.putBoolean("isElevatorAtTarget", isElevatorAtTarget);

        double tolerance = 0.01;
        double targetPositionMeters = TargetElevatorRotations * Constants.ElevatorConstants.kRotationstoMeters;

        if (Math.abs(elevatorEncoder.getPosition().getValueAsDouble() - targetPositionMeters) < tolerance
                && TargetElevatorRotations != 0d) {
            isElevatorAtTarget = true;
        }
    }
}
