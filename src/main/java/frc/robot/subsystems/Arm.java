package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private static Arm mInstance;
    private TalonFX armMotor;
    private double armAngle;
    private MotionMagicTorqueCurrentFOC armControl;
    private CANcoder armEncoder;
    private Mechanism2d simArmMechanism;
    private MechanismLigament2d simArm;
    private MechanismRoot2d simArmRoot;

    public static Arm getInstance() {
        if (mInstance == null)
            mInstance = new Arm();
        return mInstance;
    }

    public Arm() {
        armMotor = new TalonFX(Constants.ArmConstants.kArmMotorId);
        armEncoder = new CANcoder(Constants.ArmConstants.kArmEncoderId);

        var slot0Configs = new Slot0Configs();

        slot0Configs.kP = Constants.ArmConstants.kArmP;
        slot0Configs.kI = Constants.ArmConstants.kArmI;
        slot0Configs.kD = Constants.ArmConstants.kArmD;
        slot0Configs.kS = Constants.ArmConstants.kArmFF;
        slot0Configs.GravityType = Constants.ArmConstants.kArmgravtype;

        armMotor.getConfigurator().apply(slot0Configs);

        simArmMechanism = new Mechanism2d(Constants.ArmConstants.kArmWidth, Constants.ArmConstants.kArmheight);
        
        // TODO make these constants when your not being a lazy bum
        simArm = new MechanismLigament2d("Arm", 2, 270, 6, new Color8Bit(Color.kPink));

        simArmMechanism.getRoot("root",
                Constants.ArmConstants.kArmWidth / 2,
                Constants.ArmConstants.kArmheight / 2)
                .append(simArm);

        SmartDashboard.putData("Arm/Arm", simArmMechanism);

    }

    public double getArmAngle() {
        return armAngle;
    }

    public void postStatus(String status) {
        SmartDashboard.putString("Arm/status", status);

    }

    /**
     * sets the arm motor to the specific angle
     * @param angle the angle to set the arm to in Rotation2d
     */
    public void setArmMotor(Rotation2d angle) {
        if (isArmSmart(armAngle)) {
            armControl = new MotionMagicTorqueCurrentFOC(angle.getMeasure());
            armMotor.setControl(armControl);
            armAngle = angle.getDegrees();
            simArm.setAngle(armAngle);
        }

    }

    public boolean isArmSmart(double target) {
        return (target > -180) || (target < 0);
    }

    @Override
    public void periodic() {
        // TODO add logging

        // if(Robot.isSimulation()){
        // }

        // armAngle = armEncoder.getPosition().getValueAsDouble();
    }

}
