package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.commands.ElevatorMoveTo;
import org.jmhsrobotics.frc2025.commands.ScoreBargeWrist;
import org.jmhsrobotics.frc2025.commands.autoAlign.AlignBarge;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class AutoScoreAlgae extends SequentialCommandGroup {
  public AutoScoreAlgae(
      Drive drive, Elevator elevator, Wrist wrist, Intake intake, double yOffset) {
    addCommands(
        new ParallelCommandGroup(
            new AlignBarge(drive, yOffset),
            new ElevatorMoveTo(elevator, Constants.ElevatorConstants.kBargeMeters)),
        new ScoreBargeWrist(wrist, elevator, intake));
  }
}
