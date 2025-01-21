package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private static Elevator mInstance;

    // real elevator
    private TalonFX elevatorMotor;
    private double elevatorPosition;
    private CANcoder elevatorEncoder;

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

        elevatorMotorModel = DCMotor.getFalcon500(1);


        simElevator = new ElevatorSim(elevatorMotorModel, Constants.ElevatorConstants.kElevatorGearing,
                Constants.ElevatorConstants.kElevatorWeight, Constants.ElevatorConstants.kDrumRadius,
                Constants.ElevatorConstants.kMinElevatorHeight, Constants.ElevatorConstants.kmaxElevatorHeight, false,
                2d);

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
        if (TargetElevatorMeters > Constants.ElevatorConstants.kMinElevatorHeight
                && TargetElevatorMeters < Constants.ElevatorConstants.kmaxElevatorHeight) {

            double elevatorrots = TargetElevatorMeters * Constants.ElevatorConstants.kRotationstoMeters;

            MotionMagicTorqueCurrentFOC elevatorMotionMagic = new MotionMagicTorqueCurrentFOC(elevatorrots)
                    .withFeedForward(Constants.ElevatorConstants.kElevatorFF);
            elevatorMotor.setControl(elevatorMotionMagic);
        }
    }

    public double getElevatorPosition() {
        return elevatorPosition;
    }

    @Override
    public void periodic() {
        // elevatorPosition = elevatorEncoder.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Elevator/Position", simElevator.getPositionMeters());
        SmartDashboard.putNumber("Elevator/Velocity", simElevator.getVelocityMetersPerSecond());

        simElevator.setInput(elevatorMotor.getSimState().getTorqueCurrent());

        simElevator.update(0.02);

        elevatorEncoder.getSimState().setRawPosition(simElevator.getPositionMeters() * Constants.ElevatorConstants.kRotationstoMeters);

        m_elevatorMech2d.setLength(simElevator.getPositionMeters());

    }

}