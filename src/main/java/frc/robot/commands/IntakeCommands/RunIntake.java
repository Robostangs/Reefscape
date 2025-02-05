package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Endeffector;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {

    Intake intake;
    // Endeffector endeffector;
    boolean stupid;

    public RunIntake(boolean stupid) {
        intake = Intake.getInstance();
        addRequirements(intake);
        this.stupid = stupid;
    }

   

    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.postStatus("SCHLOOPING");
  }

  @Override
  public void execute() {
    if(stupid){
    intake.runIntake(0.5);
    }
    else{
      intake.runIntake(-0.5);

    }
    // endeffector.setEneffdector(0.3);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.postStatus("SCHLOOPED");
    intake.runIntake(0d);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO do this when we put it on
    // return intake.getIntakeSensor();
    return false;
  }

}


