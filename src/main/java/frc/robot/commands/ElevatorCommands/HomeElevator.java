package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class HomeElevator extends Command{

    Elevator elevator;
    public HomeElevator() {
        elevator = Elevator.getInstance();

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.postStatus("elevator homing");
    }

    @Override
    public void execute() {
        elevator.setStatorCurrentLimit(Constants.ElevatorConstants.kElevatorHomeStatorCurrentLimit);
        elevator.setElevatorDutyCycle(Constants.ElevatorConstants.kElevatorHomeDutyCycle);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorDutyCycle(0);
        elevator.setStatorCurrentLimit(Constants.ElevatorConstants.kElevatorRegStatorCurrentLimit);
        elevator.setElevatorPositionMeters(0);
        elevator.postStatus("elevator homed");
        
    }

    public boolean isFinished() {
        // return elevator.isElevatorAtHome();
        return false;
    }

    
}
