package frc.robot.commands.Factories;

import java.util.Set;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ArmCommands.SetArmPosition;
import frc.robot.commands.ElevatorCommands.SetElevatorPosition;
import frc.robot.commands.EndeffectorCommands.Slurp;
import frc.robot.commands.EndeffectorCommands.Spit;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.DeferredCommand;

public class ScoringFactory {

    public enum ScoringPosition {
        L1, L2, L3, L4, Stow, Schloop, Algaeeeee, Source
    }

    static Alert stowAlert = new Alert("Can't go to stow", Alert.AlertType.kError);
    static Alert algaeAlert = new Alert("Can't knock out algae", Alert.AlertType.kError);

    public static ScoringPosition ScoreState = ScoringPosition.Stow;

    /**
     * Returns a command that does the following commands:
     * 
     * <ul>
     * <li>Move the elevator to the limit switch position.</li>
     * <li>Move the arm to the L2 scoring position.</li>
     * <li>Move the elevator to the final L2 setpoint.</li>
     * </ul>
     * <p>
     * 
     * @return A command to move the elevator and arm to the L2 scoring position.
     */
    public static Command L2Position() {

        return new DeferredCommand(() -> {

            if (ScoreState.equals(ScoringPosition.L3) || ScoreState.equals(ScoringPosition.L4)) {
                return new SetArmPosition(Constants.ScoringConstants.L2.kArmScoringPosition - 0.1).alongWith(
                        new SetElevatorPosition(Constants.ScoringConstants.L2.kElevatorEnd))
                        .andThen(new SetArmPosition(Constants.ScoringConstants.L2.kArmScoringPosition))
                        .finallyDo(() -> {
                            ScoreState = ScoringPosition.L2;
                        });
            } else {
                if(ScoreState.equals(ScoringPosition.Stow)){
                    return (new SetElevatorPosition(Constants.ScoringConstants.L2.kElevatorStart).andThen(
                        new SetElevatorPosition(Constants.ScoringConstants.L2.kElevatorEnd)))
                    .alongWith(
                        new WaitUntilCommand(() -> Elevator.getInstance().getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                        .andThen(       
                    new SetArmPosition(Constants.ScoringConstants.L2.kArmScoringPosition)))
                        
                                    .finallyDo(() -> {
                                        ScoreState = ScoringPosition.L2;
                                    });
                }
                else{
                    return SmartStow().andThen((new SetElevatorPosition(Constants.ScoringConstants.L2.kElevatorStart).andThen(
                        new SetElevatorPosition(Constants.ScoringConstants.L2.kElevatorEnd)))
                    .alongWith(
                        new WaitUntilCommand(() -> Elevator.getInstance().getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                        .andThen(       
                    new SetArmPosition(Constants.ScoringConstants.L2.kArmScoringPosition)))
                        
                                    .finallyDo(() -> {
                                        ScoreState = ScoringPosition.L2;
                                    }));
                }

            }
        }, Set.of(Arm.getInstance(), Elevator.getInstance()));

    }

    /**
     * Returns a command that executes the following commands:
     * <p>
     * <ul>
     * <li>Move the elevator to the L3 position.</li>
     * <li>Wait until the elevator is high enough to safely move the arm.</li>
     * <li>Move the arm to the L3 scoring position.</li>
     * </ul>
     * <p>
     * 
     * @return A command to move the elevator and arm to the L3 scoring position.
     */
    public static Command L3Position() {
        return new SetElevatorPosition(Constants.ScoringConstants.L3.kElevatorPos)
                .alongWith(new WaitUntilCommand(
                        () -> Elevator.getInstance()
                                .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                        .onlyIf(() -> !Robot.isSimulation())
                        .andThen(new SetArmPosition(Constants.ScoringConstants.L3.kArmScoringPosition))
                        .finallyDo(() -> {
                            ScoreState = ScoringPosition.L3;
                        }));
    }

    /**
     * Returns a command that executes the following commands:
     * *
     * <p>
     * <ul>
     * <li>Move the elevator to the L4 position.</li>
     * <li>Wait until the elevator is high enough to safely move the arm.</li>
     * <li>Move the arm to the L4 scoring position.</li>
     * </ul>
     * <p>
     * 
     * @return A command to move the elevator and arm to the L4 scoring position.
     */
    public static Command L4Position() {
        return new SetElevatorPosition(Constants.ScoringConstants.L4.kElevatorPos)
                .alongWith(
                        new WaitUntilCommand(
                                () -> Elevator.getInstance()
                                        .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                                .onlyIf(() -> !Robot.isSimulation())
                                .andThen(
                                        new SetArmPosition(Constants.ScoringConstants.L4.kArmScoringPosition))
                                .finallyDo(() -> {
                                    ScoreState = ScoringPosition.L4;
                                }));
    }

    public static Command L4PositionAuto() {
        // return new SetElevatorPosition(Constants.ScoringConstants.Stow.kElevatorPos)
        // .andThen(new SetArmPosition(Constants.ScoringConstants.L4.kArmPosAuto))
        // .andThen(new SetElevatorPosition(Constants.ScoringConstants.L4.kElevatorPos)
        // .finallyDo(() -> {
        // ScoreState = ScoringPosition.L4;
        // }));
        return new SetElevatorPosition(Constants.ScoringConstants.L4.kElevatorPos)
                .alongWith(
                        new WaitUntilCommand(
                                () -> Elevator.getInstance()
                                        .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                                .onlyIf(() -> !Robot.isSimulation())
                                .andThen(
                                        new SetArmPosition(Constants.ScoringConstants.L4.kArmPosAuto))
                                .finallyDo(() -> {
                                    ScoreState = ScoringPosition.L4;
                                }));
    }

    public static Command L4PositionAutoStupid() {
        return new SetElevatorPosition(Constants.ScoringConstants.L4.kElevatorPos)
                .alongWith(
                        new WaitUntilCommand(
                                () -> Elevator.getInstance()
                                        .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                                .onlyIf(() -> !Robot.isSimulation())
                                .andThen(
                                        new SetArmPosition(Constants.ScoringConstants.L4.kArmPosAuto))
                                .finallyDo(() -> {
                                    ScoreState = ScoringPosition.L4;
                                }));
    }

    public static Command L3PositionAuto() {
        return new SetElevatorPosition(Constants.ScoringConstants.L3.kElevatorPos)
                .alongWith(
                        new WaitUntilCommand(
                                () -> Elevator.getInstance()
                                        .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                                .onlyIf(() -> !Robot.isSimulation())
                                .andThen(
                                        new SetArmPosition(Constants.ScoringConstants.L3.kArmPosAuto))
                                .finallyDo(() -> {
                                    ScoreState = ScoringPosition.L3;
                                }));
    }

    /**
     * Returns a command that makes the elevator go to the L3 setpoint then move the
     * arm up ot knock out algae
     * 
     * @return A command to knock out algae
     */
    public static Command ByeByeByeAlgaeL3() {

        return new SetElevatorPosition(Constants.ScoringConstants.L3.kELevatorAlgaepos)
                .andThen(new SetArmPosition(Constants.ScoringConstants.L3.kArmAlgaePos)
                        .finallyDo(() -> ScoreState = ScoringPosition.Algaeeeee));

    }

    public static Command ByeByeByeAlgaeL2() {

        return new SetElevatorPosition(Constants.ScoringConstants.L2.kElevatorStartAlg).andThen(
                new SetArmPosition(Constants.ScoringConstants.L3.kArmAlgaePos));
        // return new
        // SetElevatorPosition(Constants.ScoringConstants.L2.kELevatorAlgaepos)
        // .andThen(new SetArmPosition(Constants.ScoringConstants.L2.kArmAlgaePosStart)
        // .andThen(new WaitUntilCommand(manipBumper))
        // .andThen(new SetArmPosition(Constants.ScoringConstants.L2.kArmAlgaePosEnd))
        // .finallyDo(() -> ScoreState = ScoringPosition.Algaeeeee));

    }

    /**
     * Returns {@code L2Position()} but then spits while the bumper is held
     * 
     * @param manipBumper The Manip bumper
     * @return A command to position at L2 and then spit
     */
    public static Command L2Score(Trigger manipBumper) {
        return L2Position()
                .andThen(new WaitUntilCommand(manipBumper)).andThen(new Spit().onlyWhile(manipBumper));
    }

    /**
     * Returns {@code L3Position()} but then spits while the bumper is held
     * 
     * @param manipBumper The Manip bumper
     * @return A command to position at L3 and then spit
     */
    public static Command L3Score(Trigger manipBumper) {
        return L3Position()
                .andThen(new WaitUntilCommand(manipBumper)).andThen(new Spit().onlyWhile(manipBumper));
    }

    public static Command L3ScoreAuto() {
        return L3Position()
                .andThen(new Spit().withTimeout(0.5));
    }

    public static Command L1Position() {
        return new SetElevatorPosition(Constants.ScoringConstants.L1.kElevatorStart)
                .alongWith(
                        new WaitUntilCommand(
                                () -> Elevator.getInstance()
                                        .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                                .onlyIf(() -> !Robot.isSimulation())
                                .andThen(
                                        new SetArmPosition(Constants.ScoringConstants.L1.kArmScoringPosition))
                                .finallyDo(() -> {
                                    ScoreState = ScoringPosition.L3;
                                }));
    }

    public static Command L1Score(Trigger manipBumper) {
        return L1Position()
                .andThen(new WaitUntilCommand(manipBumper)).andThen(new Slurp().onlyWhile(manipBumper));
    }

    // -0.643
    public static Command L4ScoreAuto() {
        return L4PositionAuto()
                .andThen(new Spit().withTimeout(0.5));
    }

    // -0.643

    /**
     * Returns {@code L4Position()} but then spits while the bumper is held
     * 
     * @param manipBumper The Manip bumper
     * @return A command to position at L4 and then spit
     */
    public static Command L4Score(Trigger manipBumper) {
        return L4Position()
                .andThen(new WaitUntilCommand(manipBumper)).andThen(new Spit().onlyWhile(manipBumper));
    }

    public static Command AutoL4Score() {
        return new SetElevatorPosition(Constants.ScoringConstants.L4.kElevatorPos)
                .alongWith(
                        new WaitUntilCommand(
                                () -> Elevator.getInstance()
                                        .getElevatorPositionMeters() > Constants.ElevatorConstants.kSafeArmElevatorPosition)
                                .onlyIf(() -> !Robot.isSimulation())
                                .andThen(
                                        // TODO Find this setpoint
                                        new SetArmPosition(Constants.ScoringConstants.L4.kArmAutoScoringPosition))
                                .finallyDo(() -> {
                                    ScoreState = ScoringPosition.L4;
                                }));
    }

    /**
     * A command that executes the following commands:
     * *
     * <p>
     * <ul>
     * <li>Move the elevator to the stow position.</li>
     * <li>Move the arm to the stow position.</li>
     * </ul>
     * <p>
     * 
     * @return A command to stow at L2
     */
    public static Command StowL2() {
        return new SetElevatorPosition(Constants.ScoringConstants.Stow.kElevatorPos)
                .andThen(new SetArmPosition(Constants.ScoringConstants.Stow.kArmStowPos));
    }

    /**
     * A command that executes the following commands:
     * <p>
     * <ul>
     * *
     * <li>Move the arm to the stow position.</li>
     * <li>Move the elevator to the stow position.</li>
     * </ul>
     * <p>
     * 
     * @return A command to stow
     */
    public static Command Stow() {
        return new SetArmPosition(Constants.ArmConstants.kArmRestSetpoint).andThen(
                new SetElevatorPosition(Constants.ScoringConstants.Stow.kElevatorPos));
    }

    /**
     * Returns a command that decides whether to stow at L2 or perform a regular
     * stow based on the current scoring position.
     *
     * @return A command to stow the robot based on the current scoring position.
     */
    public static Command SmartStow() {
        return new DeferredCommand(() -> {
            if (ScoreState.equals(ScoringPosition.L2)) {
                return StowL2().finallyDo(() -> {
                    ScoreState = ScoringPosition.Stow;
                });
            } else {
                return Stow().finallyDo(() -> {
                    ScoreState = ScoringPosition.Stow;
                });
            }
        }, Set.of(Arm.getInstance(), Elevator.getInstance()));

    }

    /**
     * Returns a command that does the following commands:
     * <p>
     * <ul>
     * <li>Move the elevator to the source position.</li>
     * <li>Move the arm to the source position.</li>
     * </ul>
     * <p>
     * 
     * @return A command intake from source if ground intake ever breaks
     */
    public static Command SourceIntake() {
        return new SetElevatorPosition(Constants.ScoringConstants.Source.kElevatorPos)
                .andThen(new SetArmPosition(Constants.ScoringConstants.Source.kArmSourcePosition)
                        .finallyDo(() -> ScoreState = ScoringPosition.Source));
    }

    /**
     * Returns a command that does the following commands:
     * <p>
     * <ul>
     * <li>Move the elevator to the schloop position.</li>
     * <li>Move the arm to the schloop position.</li>
     * </ul>
     * <p>
     * 
     * @return A command to move the elevator and arm to get the coral
     */
    public static Command Schloop() {
        return new SetArmPosition(Constants.ScoringConstants.Stow.kArmStowPos).andThen(
                new SetElevatorPosition(Constants.ScoringConstants.Schloop.kElevatorPos).finallyDo(() -> {
                    ScoreState = ScoringPosition.Schloop;
                }));

    }

    public static Runnable returnStow() {
        return () -> {
            SmartStow();
        };
    }

    public static Runnable returnStowL2() {
        return () -> {
            StowL2();
        };
    }

    public static Runnable getSchloop() {
        return () -> {
            Schloop();
        };
    }
}