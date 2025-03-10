package org.jmhsrobotics.frc2025.commands;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class ElevatorAndWristMove extends SequentialCommandGroup {

  public ElevatorAndWristMove(
      Elevator elevator,
      Wrist wrist,
      Intake intake,
      double elevatorGoalMeters,
      double wristGoalDegrees) {
    addCommands(
        // Moves wrist to safe position if it is going to hit elevator frame otherwise
        new WristMoveTo(wrist, Constants.WristConstants.kSafeAngleDegrees)
            .onlyIf(() -> wrist.getPositionDegrees() <= Constants.WristConstants.kSafeAngleDegrees),
        // If elevator is in L4 position, wrist will go to safe position before elevator to prevent
        // wrist from hitting the reef
        new WristMoveTo(wrist, Constants.WristConstants.kSafeAngleDegrees)
            .onlyIf(() -> elevator.getSetpoint() == Constants.ElevatorConstants.kLevel4Meters),
        // If elevator was at barge and still as algae, rotate out first
        new WristMoveTo(wrist, Constants.WristConstants.kRotationAlgaeDegrees)
            .onlyIf(
                () ->
                    elevator.getSetpoint() == Constants.ElevatorConstants.kBargeMeters
                        && intake.getMode() == 1),
        new ElevatorMoveTo(elevator, elevatorGoalMeters),
        new WristMoveTo(wrist, wristGoalDegrees).withTimeout(3));
  }
}
