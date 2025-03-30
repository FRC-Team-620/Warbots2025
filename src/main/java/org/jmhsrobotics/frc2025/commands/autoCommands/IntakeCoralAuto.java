package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.commands.ElevatorAndWristMove;
import org.jmhsrobotics.frc2025.commands.IndexerMove;
import org.jmhsrobotics.frc2025.commands.IntakeFromIndexer;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class IntakeCoralAuto extends SequentialCommandGroup {
  public IntakeCoralAuto(Elevator elevator, Wrist wrist, Intake intake, LED led, Indexer indexer) {
    addCommands(
        new ElevatorAndWristMove(
            elevator,
            wrist,
            intake,
            Constants.ElevatorConstants.kCoralIntakeMeters,
            Constants.WristConstants.kSafeAngleDegrees),
        new ParallelRaceGroup(
            new IntakeFromIndexer(wrist, intake, led), new IndexerMove(indexer, intake)));
  }
}
