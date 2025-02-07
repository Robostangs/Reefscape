package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
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

        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = Constants.ArmConstants.kArmP;
        slot0Configs.kI = Constants.ArmConstants.kArmI;
        slot0Configs.kD = Constants.ArmConstants.kArmD;
        slot0Configs.kS = Constants.ArmConstants.kArmFF;
        slot0Configs.GravityType = Constants.ArmConstants.kArmgravtype;

        armControl.Slot = 0;
        TalonFXConfiguration armconfigs = new TalonFXConfiguration();

        armMotor.getConfigurator().apply(slot0Configs);

        armMechanism = new Mechanism2d(Constants.ArmConstants.kArmWidth, Constants.ArmConstants.kArmheight);

        arm = new MechanismLigament2d("Arm", 2, 270, 6, new Color8Bit(Color.kOrange));

        armMechanism.getRoot("Root",
                Constants.ArmConstants.kArmWidth / 2,
                Constants.ArmConstants.kArmheight / 2)
                .append(arm);

        targetArmMechanism = new Mechanism2d(Constants.ArmConstants.kArmWidth, Constants.ArmConstants.kArmheight);

        // TODO make these constants when your not being a lazy bum
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
     * @param angle the angle to set the arm to in Rotation2d
     */
    public void setArmMotor(double angle) {
        // if (isArmSmart(targetArmAngle)) {
        armControl.Position = Units.degreesToRotations(angle);
        // }

    }

    public boolean isArmSmart(double target) {
        return (target > -180) || (target < 0);
    }

    public void setArmPosition() {

        if (Robot.isSimulation()) {
            armEncoder.getSimState().setRawPosition(targetArmAngle);
            targetArm.setAngle(targetArmAngle);
        } else {
            arm.setAngle(armEncoder.getPosition().getValueAsDouble());

        }
    }

    @Override
    public void periodic() {

        setArmPosition();

        SmartDashboard.putNumber("target arm angle", targetArmAngle);
        SmartDashboard.putNumber("actual arm angle", armEncoder.getPosition().getValueAsDouble());

        armMotor.setControl(armControl);

    }

}
