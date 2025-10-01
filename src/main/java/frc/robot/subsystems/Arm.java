package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Arm extends SubsystemBase {

    //Creates the variables in the subsystem
    private static Arm mInstance;
    private TalonFX armMotor;
    private MotionMagicTorqueCurrentFOC armControl;
    private CANcoder armEncoder;

    private Mechanism2d armMechanism;
    private MechanismLigament2d arm;

    private Mechanism2d targetArmMechanism;
    private MechanismLigament2d targetArm;
    private Alert armPastRotation;
    //Function to get the active instance of the Arm
    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }
        return mInstance;
    }
    // Define the variables in the subsystem.
    public Arm() {
        armMotor = new TalonFX(Constants.ArmConstants.kArmMotorId);
        armEncoder = new CANcoder(Constants.ArmConstants.kArmEncoderId);
        armControl = new MotionMagicTorqueCurrentFOC(Constants.ArmConstants.kArmRestSetpoint);

        armPastRotation = new Alert("Arm is past 1 rotation and will tweak if it goes to setpoint", AlertType.kWarning);

        armControl.Slot = 0;
        TalonFXConfiguration armconfigs = new TalonFXConfiguration();

        armconfigs.Slot0.kP = Constants.ArmConstants.kArmP;
        armconfigs.Slot0.kI = Constants.ArmConstants.kArmI;
        armconfigs.Slot0.kD = Constants.ArmConstants.kArmD;
        armconfigs.Slot0.kS = Constants.ArmConstants.kArmS;
        armconfigs.Slot0.GravityType = Constants.ArmConstants.kArmgravtype;
        armconfigs.Slot0.kG = Constants.ArmConstants.kArmG;
        armconfigs.Slot0.kA = Constants.ArmConstants.kArmA;
        armconfigs.Slot0.kV = Constants.ArmConstants.kArmV;

        armconfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmConstants.kArmCruiseVelocity;
        armconfigs.MotionMagic.MotionMagicAcceleration = Constants.ArmConstants.kArmAcceleration;

        armconfigs.Feedback.FeedbackRemoteSensorID = Constants.ArmConstants.kArmEncoderId;
        armconfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        armconfigs.Feedback.RotorToSensorRatio = Constants.ArmConstants.kArmRotortoSensorRatio;

        armMotor.getConfigurator().apply(armconfigs);

        armMechanism = new Mechanism2d(Constants.ArmConstants.kArmWidth, Constants.ArmConstants.kArmHeight);

        arm = new MechanismLigament2d("Arm", 2, 270, 6, new Color8Bit(Color.kOrange));

        armMechanism.getRoot("Root",
                Constants.ArmConstants.kArmWidth / 2,
                Constants.ArmConstants.kArmHeight / 2)
                .append(arm);

        targetArmMechanism = new Mechanism2d(Constants.ArmConstants.kArmWidth, Constants.ArmConstants.kArmHeight);

        targetArm = new MechanismLigament2d("Target Arm", 2, 270, 6, new Color8Bit(Color.kBlue));

        targetArmMechanism.getRoot("Target Root",
                Constants.ArmConstants.kArmWidth / 2,
                Constants.ArmConstants.kArmHeight / 2)
                .append(targetArm);

        SmartDashboard.putData("Arm/Arm", armMechanism);
        SmartDashboard.putData("Arm/TargetArm", targetArmMechanism);
        
    }
    
    /**
     * Posts the status given to the SmartDashboard
     * 
     * @param status What the message says
     */
    public void postStatus(String status) {
        SmartDashboard.putString("Arm/status", status);

    }

    /**
     * Function to set the arm to the angle given
     * 
     * @param rotations the angle to set the arm to in degrees
     * @return 
     */
    public void setArmPosition(double rotations) {
        armControl.Position = (rotations);
            
    }
    /**
     * Sets the arm motor to a specific duty cycle
     */
    public void setArmDutyCycle(double armDutyCycle) {
        armMotor.set(armDutyCycle);
    }

    public boolean isArmSmart(double target) {
        return (target > -180) || (target < 0);
    }

    public void setArmPosition() {
        
        if (Robot.isSimulation()) {
            armEncoder.getSimState().setRawPosition(armControl.Position);

            targetArm.setAngle(Units.rotationsToDegrees(armControl.Position));
            
        } else {
            arm.setAngle(armEncoder.getPosition().getValueAsDouble());

        }
    }
    /**
     * 
     * 
     * @return A command to move the elevator and arm to the L3 scoring position.
     */ 
    public Runnable gotoZero = () -> {
        armMotor.setControl(new MotionMagicTorqueCurrentFOC(0.15));
    };
    //Go to schloop   
    public Runnable gotoSchloop = () -> {
        armMotor.setControl(new MotionMagicTorqueCurrentFOC(-0.25));
    };
    
     /**
     * Checks if the arm is at the target position
     * @param tolerence the maximum rotations the arm can be to be considered finished
     */
    public boolean isArmAtTarget(double tolerence) {
        if (Math.abs(armControl.Position - armEncoder.getPosition().getValueAsDouble()) < tolerence) {
            return true;
        } else {
            return false;
        }
    }
    /**
     * Sets the arm to be controlled by motion magic
     */
    public void setArmMotionMagic() {

        armMotor.setControl(armControl);

    }
    
    @Override
    public void periodic() {
        // Checks if the arm is past the rotation set.
        
        if (armMotor.getPosition().getValueAsDouble() > 0.5) {
            armPastRotation.set(true);
        } else {
            armPastRotation.set(false);
            if(Robot.isSimulation()){
                arm.setAngle(Units.rotationsToDegrees(armEncoder.getPosition().getValueAsDouble()));
            }
            setArmPosition();
            SmartDashboard.putNumber("Arm/target arm angle", armControl.Position);
            SmartDashboard.putNumber("Arm/actual arm angle", armEncoder.getPosition().getValueAsDouble());

        }

    }
}
