package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

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
    private TorqueCurrentFOC elevatorControl;
    private CANcoder elevatorEncoder;

    private MotionMagicTorqueCurrentFOC elevatorMotionMagic;
    // simulated elevator
    private ElevatorSim simElevator;
    private DCMotor elevatorMotorModel;
    private Mechanism2d simElevatorMechanism;
    private MechanismRoot2d simElevatorRoot;
    private MechanismRoot2d simElevatorRoot2;
    private MechanismRoot2d simPlatELevator;

    private MechanismLigament2d simElevatorLeft;
    private MechanismLigament2d simElevatorRight;
    private MechanismLigament2d simElevatorIntake;

    private EncoderSim elevatorSimEncoder;

    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d m_mech2d = new Mechanism2d(1, 1);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 0.3, 0);
    private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
            new MechanismLigament2d("Elevator", 0.3, 90));

    public static Elevator getInstance() {
        if (mInstance == null)
            mInstance = new Elevator();
        return mInstance;
    }

    public Elevator() {
        elevatorMotor = new TalonFX(Constants.ElevatorConstants.kElevatorMotorId);
        elevatorControl = new TorqueCurrentFOC(Constants.ElevatorConstants.kElevatorMaxCurrent);
        // elevatorMotorModel = DCMotor.getFalcon500(1);
        // // TODO find gearing of the elevator
        // simElevator = new ElevatorSim(elevatorMotorModel,
        // Constants.ElevatorConstants.kElevatorGearing,
        // Constants.ElevatorConstants.kElevatorWeight,
        // Constants.ElevatorConstants.kDrumRadius,
        // Constants.ElevatorConstants.kminElevatorHeight,
        // Constants.ElevatorConstants.kmaxElevatorHeight,
        // false, 1.1);

        // simElevatorMechanism = new
        // Mechanism2d(Constants.ElevatorConstants.kElevatorHeight,
        // Constants.ElevatorConstants.kElevatorWidth);

        // simElevatorRoot = simElevatorMechanism.getRoot("Elevator",
        // Constants.ElevatorConstants.kRootElevatorX,
        // Constants.ElevatorConstants.kRootElevatorY);
        // simPlatELevator = simElevatorMechanism.getRoot("Plat",
        // Constants.ElevatorConstants.kRootElevator2X,
        // elevatorPosition);
        // simElevatorRoot2 = simElevatorMechanism.getRoot("Elevator2",
        // Constants.ElevatorConstants.kRootElevator2X,
        // Constants.ElevatorConstants.kRootElevator2Y);

        // simElevatorLeft = simElevatorRoot
        // .append(new MechanismLigament2d("Liga1",
        // Constants.ElevatorConstants.kLigaLength, 90));
        // simElevatorIntake = simPlatELevator
        // .append(new MechanismLigament2d("Intake",
        // Constants.ElevatorConstants.kLigaLength, 0));
        // simElevatorRight = simElevatorRoot2
        // .append(new MechanismLigament2d("Liga2",
        // Constants.ElevatorConstants.kLigaLength, 90));

        var slot0Configs = new Slot0Configs();
        // TODO tune these values
        slot0Configs.kP = Constants.ElevatorConstants.kElevatorP;
        slot0Configs.kI = Constants.ElevatorConstants.kElevatorI;
        slot0Configs.kD = Constants.ElevatorConstants.kElevatorD;
        slot0Configs.kS = Constants.ElevatorConstants.kElevatorFF;

        elevatorMotor.getConfigurator().apply(slot0Configs);

    }

    public void setElevatorPosition(double TargetElevatorMeters) {
        if (TargetElevatorMeters > Constants.ElevatorConstants.kminElevatorHeight
                && TargetElevatorMeters < Constants.ElevatorConstants.kmaxElevatorHeight) {

            double elevatorrots = TargetElevatorMeters * Constants.ElevatorConstants.kRotationstoMeters;


            elevatorMotor.setControl(elevatorMotionMagic.withFeedForward(Constants.ElevatorConstants.kElevatorFF)
                    .withPosition(elevatorrots));
                        }        }

    

    public void setSimElevatorPosition(double TargetElevatorMeters) {
        if (TargetElevatorMeters > Constants.ElevatorConstants.kminElevatorHeight
                && TargetElevatorMeters < Constants.ElevatorConstants.kmaxElevatorHeight) {

            double elevatorrots = TargetElevatorMeters * Constants.ElevatorConstants.kRotationstoMeters;

            elevatorMotor.setControl(elevatorMotionMagic.withFeedForward(Constants.ElevatorConstants.kElevatorFF)
                    .withPosition(elevatorrots));
                        }
    }

    public double getElevatorPosition() {
        return elevatorPosition;
    }

    @Override
    public void periodic() {
        elevatorPosition = elevatorEncoder.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("Elevator/position", elevatorPosition);

        SmartDashboard.putData("Elevator Sim", m_mech2d);

    }

}