package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class HomeElevator extends Command {

    Elevator elevator;

    /**
     * Runs the elevator at a duty cycle until it hits limit switch
     */
    public HomeElevator() {
        elevator = Elevator.getInstance();

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.postStatus("elevator homing");
        elevator.setElevatorDutyCycle(Constants.ElevatorConstants.kElevatorHomeDutyCycle);

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorDutyCycle(0);
        elevator.setElevatorPosition(Constants.ElevatorConstants.kHomePosition);
        elevator.postStatus("elevator homed");
        elevator.isHome = true;

    }

    public boolean isFinished() {
        return !elevator.getLimitSwitch();
    }

}
