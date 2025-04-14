package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2025.Robot;
import org.jmhsrobotics.frc2025.commands.ElevatorAndWristMove;
import org.jmhsrobotics.frc2025.commands.FixCoralPlacement;
import org.jmhsrobotics.frc2025.commands.IntakeFromIndexer;
import org.jmhsrobotics.frc2025.commands.autoAlign.AlignReefSetAngle;
import org.jmhsrobotics.frc2025.commands.autoAlign.AlignReefSetAngleProf;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class AutoScoreCoral extends SequentialCommandGroup {
  public AutoScoreCoral(
      Drive drive,
      Elevator elevator,
      Wrist wrist,
      Intake intake,
      Indexer indexer,
      Vision vision,
      LED led,
      boolean isLeft,
      int targetTagID) {
    addCommands(
        new ParallelCommandGroup(
            new AlignReefSetAngle(drive, vision, led, elevator, isLeft, targetTagID),
            // intakes from indexer, timeout dependent on if sim or real, also fixes coral placement
            // and then raises the elevator and moves wrist
            new SequentialCommandGroup(
                new IntakeFromIndexer(wrist, intake, indexer, led)
                    .withTimeout(2)
                    .onlyIf(() -> Robot.isSimulation()),
                new IntakeFromIndexer(wrist, intake, indexer, led)
                    .withTimeout(8)
                    .onlyIf(() -> Robot.isReal()),
                new ParallelCommandGroup(
                    new FixCoralPlacement(intake).withTimeout(2),
                    new ElevatorAndWristMoveAlt(elevator, wrist)))),
        // scores coral. moves wrist in parallel if already aligned
        new ScoreCoral(intake).withTimeout(0.15));
  }

  // works the same as with the other constructor but allows for any wrist/elevator goal for L2 and
  // L3
  public AutoScoreCoral(
      Drive drive,
      Elevator elevator,
      Wrist wrist,
      Intake intake,
      Indexer indexer,
      Vision vision,
      LED led,
      boolean isLeft,
      int targetTagID,
      double elevatorGoalMeters,
      double wristGoalDegrees) {
    addCommands(
        new ParallelCommandGroup(
            new AlignReefSetAngle(drive, vision, led, elevator, isLeft, targetTagID),
            new SequentialCommandGroup(
                new IntakeFromIndexer(wrist, intake, indexer, led)
                    .withTimeout(2)
                    .onlyIf(() -> Robot.isSimulation()),
                new IntakeFromIndexer(wrist, intake, indexer, led)
                    .withTimeout(8)
                    .onlyIf(() -> Robot.isReal()),
                new ParallelCommandGroup(
                    new FixCoralPlacement(intake).withTimeout(2),
                    new ElevatorAndWristMove(
                        elevator, wrist, elevatorGoalMeters, wristGoalDegrees)))),
        new ScoreCoral(intake).withTimeout(0.15));
  }

  public AutoScoreCoral(
      Drive drive,
      Elevator elevator,
      Wrist wrist,
      Intake intake,
      Indexer indexer,
      Vision vision,
      LED led,
      boolean isLeft,
      int targetTagID,
      boolean isProfiled) {
    addCommands(
        new ParallelCommandGroup(
            new AlignReefSetAngleProf(drive, vision, led, elevator, isLeft, targetTagID),
            new SequentialCommandGroup(
                new IntakeFromIndexer(wrist, intake, indexer, led)
                    .withTimeout(2)
                    .onlyIf(() -> Robot.isSimulation()),
                new IntakeFromIndexer(wrist, intake, indexer, led)
                    .withTimeout(8)
                    .onlyIf(() -> Robot.isReal()),
                new ParallelCommandGroup(
                    new FixCoralPlacement(intake).withTimeout(2),
                    new ElevatorAndWristMoveAlt(elevator, wrist)))),
        new ScoreCoral(intake).withTimeout(0.15));
  }
}
