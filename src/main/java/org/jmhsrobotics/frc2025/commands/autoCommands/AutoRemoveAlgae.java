package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.commands.ElevatorAndWristMove;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class AutoRemoveAlgae extends ParallelCommandGroup {
  public AutoRemoveAlgae(
      Drive drive,
      Elevator elevator,
      Wrist wrist,
      Intake intake,
      Vision vision,
      int targetTagID,
      double elevatorGoalMeters) {
    addCommands(
        new RemoveAlgaeAutoAlign(drive, vision, targetTagID),
        new ElevatorAndWristMove(
            elevator, wrist, elevatorGoalMeters, Constants.WristConstants.kRotationAlgaeDegrees),
        Commands.run(() -> intake.set(-0.6), intake));
  }
}
