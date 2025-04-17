package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

// moves elevator and wrist sequentially, but finishes when the elevator reaches goal. For use in
// autos
public class ElevatorAndWristMoveAlt extends Command {
  private Elevator elevator;
  private Wrist wrist;

  public ElevatorAndWristMoveAlt(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;

    addRequirements(elevator, wrist);
  }

  @Override
  public void initialize() {
    elevator.setSetpoint(Constants.ElevatorConstants.kLevel4Meters);
  }

  @Override
  public boolean isFinished() {
    return elevator.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    wrist.setSetpoint(Constants.WristConstants.kLevel4Degrees);
  }
}
