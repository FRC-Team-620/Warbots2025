package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2025.Robot;
import org.jmhsrobotics.frc2025.commands.FixCoralPlacement;
import org.jmhsrobotics.frc2025.commands.IntakeFromIndexer;
import org.jmhsrobotics.frc2025.commands.autoAlign.AlignReefSetAngle;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class AutoScoreCoral extends SequentialCommandGroup {
  // aligns reef while:
  // sequentially finishing the coral intake, raising the elevator and fixing coral placement
  // deadline group, waits on the elevator to be at the top and align reef to be finished
  // wrist should move out after elevator is done if align reef is not already done.
  // after align reef and elevator are done, score coral and move wrist to l4 setpoint are run in
  // parallel
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
        // moves wrist to L4 setpoint only after elevator has reached L4 goal, and only if the
        // commands withint "withDeadline" have not yet been finished
        // aligns reef while finishing the intake and moving up the elevator as soon as
        // possible
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
                    new ElevatorAndWristMoveAlt(elevator, wrist)))),
        // scpres coral while moving wrist out
        new ScoreCoral(intake).withTimeout(0.15));
  }
}
