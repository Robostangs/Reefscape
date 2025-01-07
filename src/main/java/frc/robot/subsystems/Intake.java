package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake {
    PWMTalonFX intakeMotortop, intakeMotorbottom;
    Alert intakeAlert = new Alert("INTAKE TWEAKING", Alert.AlertType.kError);
    private static Intake mInstance;

    public static Intake getInstance() {
        if (mInstance == null)
            mInstance = new Intake();
        return mInstance;
    }

    public Intake() {

        intakeMotortop = new PWMTalonFX(Constants.IntakeConstants.kTopIntakeMotorId);
        intakeMotorbottom = new PWMTalonFX(Constants.IntakeConstants.kBottomIntakeMotorId);

        intakeMotortop.addFollower(intakeMotorbottom);

        intakeMotortop.setInverted(Constants.IntakeConstants.kTopIntakeMotorInverted);
        intakeMotorbottom.setInverted(Constants.IntakeConstants.kBottomIntakeMotorInverted);

    }

    public void runIntake(double speed) {
        intakeMotortop.set(speed);
        // intakeMotorbottom.set(speed);
    }

    public void stopIntake() {
        intakeMotortop.stopMotor();
        // intakeMotorbottom.stopMotor();
    }

    public void periodic() {
        //TODO add logging

    }
}
