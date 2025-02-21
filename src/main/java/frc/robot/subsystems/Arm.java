package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Arm extends SubsystemBase {
    private static Arm mInstance;
    private TalonFX armMotor;
    private double targetArmAngle;
    private MotionMagicTorqueCurrentFOC armControl;
    private CANcoder armEncoder;

    private Mechanism2d armMechanism;
    private MechanismLigament2d arm;

    private Mechanism2d targetArmMechanism;
    private MechanismLigament2d targetArm;

    public static Arm getInstance() {
        if (mInstance == null)
            mInstance = new Arm();
        return mInstance;
    }

    public Arm() {
        armMotor = new TalonFX(Constants.ArmConstants.kArmMotorId);
        armEncoder = new CANcoder(Constants.ArmConstants.kArmEncoderId);
        armControl = new MotionMagicTorqueCurrentFOC(0d);

        armControl.Slot = 0;
        TalonFXConfiguration armconfigs = new TalonFXConfiguration();

        armconfigs.Slot0.kP = Constants.ArmConstants.kArmP;
        armconfigs.Slot0.kI = Constants.ArmConstants.kArmI;
        armconfigs.Slot0.kD = Constants.ArmConstants.kArmD;
        armconfigs.Slot0.kS = Constants.ArmConstants.kArmkS;
        armconfigs.Slot0.GravityType = Constants.ArmConstants.kArmgravtype;
        armconfigs.Slot0.kG = Constants.ArmConstants.kArmG;
        armconfigs.Slot0.kA = Constants.ArmConstants.kArmA;
        armconfigs.Slot0.kV = Constants.ArmConstants.kArmV;

        armconfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmConstants.kArmCruiseVelocity;
        armconfigs.MotionMagic.MotionMagicAcceleration = Constants.ArmConstants.kArmAcceleration;

        armconfigs.Feedback.FeedbackRemoteSensorID = Constants.ArmConstants.kArmEncoderId;
        armconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        armconfigs.Feedback.RotorToSensorRatio = Constants.ArmConstants.kArmRotortoSensorRatio;

        armconfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        armMotor.getConfigurator().apply(armconfigs);

        armMechanism = new Mechanism2d(Constants.ArmConstants.kArmWidth, Constants.ArmConstants.kArmheight);

        arm = new MechanismLigament2d("Arm", 2, 270, 6, new Color8Bit(Color.kOrange));

        armMechanism.getRoot("Root",
                Constants.ArmConstants.kArmWidth / 2,
                Constants.ArmConstants.kArmheight / 2)
                .append(arm);

        targetArmMechanism = new Mechanism2d(Constants.ArmConstants.kArmWidth, Constants.ArmConstants.kArmheight);

        targetArm = new MechanismLigament2d("Target Arm", 2, 270, 6, new Color8Bit(Color.kBlue));

        targetArmMechanism.getRoot("Target Root",
                Constants.ArmConstants.kArmWidth / 2,
                Constants.ArmConstants.kArmheight / 2)
                .append(targetArm);

        SmartDashboard.putData("Arm/Arm", armMechanism);
        SmartDashboard.putData("Arm/TargetArm", targetArmMechanism);

    }

    public double getTargetArmAngle() {
        return targetArmAngle;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Arm/status", status);

    }

    /**
     * sets the arm motor to the specific angle
     * 
     * @param angle the angle to set the arm to in degrees
     */
    public void setArmPosition(double angle) {
        armControl.Position = (angle);

    }

    public void setArmDutyCycle(double armDutyCycle) {
        armMotor.set(armDutyCycle);
    }

    public boolean isArmSmart(double target) {
        return (target > -180) || (target < 0);
    }

    public void setBrakeMode() {
        armMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setArmPosition() {

        if (Robot.isSimulation()) {
            armEncoder.getSimState().setRawPosition(armControl.Position);
            targetArm.setAngle(armControl.Position);
        } else {
            arm.setAngle(armEncoder.getPosition().getValueAsDouble());

        }
    }

    public Runnable gotoZero = () -> {
        armMotor.setControl(new MotionMagicTorqueCurrentFOC(0.15));
    };
    public Runnable gotoSchloop = () -> {
        armMotor.setControl(new MotionMagicTorqueCurrentFOC(-0.25));
    };

    public void setArmMotionMagic() {
        if (Intake.getInstance().getIntakePosition() <= Constants.IntakeConstants.kRetractSetpoint
                || Robot.isSimulation()) {
            armMotor.setControl(armControl);
        } else {
            postStatus("cant move, intake in way");
        }

    }

    /**
     * torque current = kg +kv*v
     * 
     * + ka*a
     * 
     */
    @Override
    public void periodic() {

        setArmPosition();
        SmartDashboard.putNumber("Arm/target arm angle", armControl.Position);
        SmartDashboard.putNumber("Arm/actual arm angle", armEncoder.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("Arm-Test/", armMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Arm-Test/Torque current", armMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Arm-Test/Velocity", armMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Arm-Test/Acceleration", armMotor.getAcceleration().getValueAsDouble());

        SmartDashboard.putNumber("Torque current  -kg ",
                (armMotor.getTorqueCurrent().getValueAsDouble()
                        - (Constants.ArmConstants.kArmG
                                * Math.cos(Units.rotationsToRadians(armMotor.getPosition().getValueAsDouble())))
                // / armMotor.getVelocity().getValueAsDouble()
                ));

        SmartDashboard.putNumber("Torque current over velocity ",
                Math.min(100, Math.max(-100, armMotor.getTorqueCurrent().getValueAsDouble()
                                / armMotor.getVelocity().getValueAsDouble())));

    }

}
