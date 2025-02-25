package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeWheels;

public class RunIntake extends Command {

    IntakeWheels intake;

    public RunIntake() {
        intake = IntakeWheels.getInstance();
        addRequirements(intake);
    }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.postStatus("Intaking");
  }

  @Override
  public void execute() {

    intake.runIntake(0.75);


  }

  @Override
  public void end(boolean interrupted) {
    intake.postStatus("Coral In");
    intake.runIntake(0d);


  }

  @Override
  public boolean isFinished() {
    if(Robot.isSimulation()){
      return false;
    }
    else{
      return intake.getIntakeSensor();
    }   
  }

}


