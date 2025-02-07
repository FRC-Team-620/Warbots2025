package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class ElevatorAndWristMove extends ParallelCommandGroup {
  // private Elevator elevator;
  // private Wrist wrist;

  // private double elevatorGoalMeters;
  // private double wristGoalDegrees;

  public ElevatorAndWristMove(
      Elevator elevator, Wrist wrist, double elevatorGoalMeters, double wristGoalDegrees) {
    addCommands(
        new SequentialCommandGroup(
            new WristMoveTo(wrist, Constants.WristConstants.kSafeAngle),
            new ElevatorMoveTo(elevator, elevatorGoalMeters),
            new WristMoveTo(wrist, wristGoalDegrees)));
  }
}
