package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.commands.ElevatorAndWristMove;
import org.jmhsrobotics.frc2025.commands.autoAlign.AlignSource;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class AutoIntakeCoral extends ParallelRaceGroup {
  public AutoIntakeCoral(
      Drive drive,
      Wrist wrist,
      Elevator elevator,
      Intake intake,
      Indexer indexer,
      LED led,
      boolean alignCloseToStation) {
    addCommands(
        // Moves elevator down while continuing to score in case coral has not been fully extaked
        // while aligning source and then beginning intake until coral is in indexer
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ElevatorAndWristMove(
                    elevator,
                    wrist,
                    Constants.ElevatorConstants.kCoralIntakeMeters,
                    Constants.WristConstants.kRotationIntakeCoralDegrees),
                new ScoreCoral(intake, elevator).withTimeout(0.15)),
            new IntakeUntilCoralInIndexer(wrist, intake, indexer, led).withTimeout(6)),
        new SequentialCommandGroup(
            new AlignSource(drive, alignCloseToStation), new DriveBackwards(drive)));
  }
}
