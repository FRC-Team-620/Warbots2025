package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;

public class ElevatorMoveTo extends Command {
  private Elevator elevatorSubsystem;
  private double goalMeters;

  public ElevatorMoveTo(Elevator elevatorSubsystem, Intake intake, int button) {
    this.elevatorSubsystem = elevatorSubsystem;
    setGoal(intake, button);

    addRequirements(elevatorSubsystem);
  }

  /**
   * Determines the height goal of the elevator command based on the mode(from the intake subsystem)
   * and which button was pressed
   *
   * @param intake
   * @param button The button pushed on the controller
   */
  private void setGoal(Intake intake, int button) {
    switch (intake.getMode()) {
        // height determined based on button pushed and control mode
      case Constants.ModeConstants.kAlgae:
        if (button == Constants.ButtonConstants.kA || button == Constants.ButtonConstants.kB)
          this.goalMeters = Constants.ElevatorConstants.kProcesserMeters;
        else this.goalMeters = Constants.ElevatorConstants.kBargeMeters;

        break;

      case Constants.ModeConstants.kSearch:
        if (button == Constants.ButtonConstants.kA)
          this.goalMeters = Constants.ElevatorConstants.kCoralIntakeMeters;
        else if (button == Constants.ButtonConstants.kB)
          this.goalMeters = Constants.ElevatorConstants.kAlgaeIntakeL2;
        else if (button == Constants.ButtonConstants.kX)
          this.goalMeters = Constants.ElevatorConstants.kAlgaeIntakeL3;
        else this.goalMeters = Constants.ElevatorConstants.kAlgaeQTipMeters;

        break;

      case Constants.ModeConstants.kCoral:
        if (button == Constants.ButtonConstants.kA)
          this.goalMeters = Constants.ElevatorConstants.kCoralIntakeMeters;
        else if (button == Constants.ButtonConstants.kB)
          this.goalMeters = Constants.ElevatorConstants.kAlgaeIntakeL2;
        else if (button == Constants.ButtonConstants.kX)
          this.goalMeters = Constants.ElevatorConstants.kAlgaeIntakeL3;
        else this.goalMeters = Constants.ElevatorConstants.kAlgaeQTipMeters;

        break;

      default:
        if (button == Constants.ButtonConstants.kA)
          this.goalMeters = Constants.ElevatorConstants.kCoralIntakeMeters;
        else if (button == Constants.ButtonConstants.kB)
          this.goalMeters = Constants.ElevatorConstants.kAlgaeIntakeL2;
        else if (button == Constants.ButtonConstants.kX)
          this.goalMeters = Constants.ElevatorConstants.kAlgaeIntakeL3;
        else this.goalMeters = Constants.ElevatorConstants.kAlgaeQTipMeters;

        break;
    }
  }

  @Override
  public void initialize() {
    this.elevatorSubsystem.setSetpoint(goalMeters);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return elevatorSubsystem.atGoal(this.goalMeters);
  }
}
