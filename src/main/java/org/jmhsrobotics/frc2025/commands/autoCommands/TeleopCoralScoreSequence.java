package org.jmhsrobotics.frc2025.commands.autoCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.drive.Drive;
import org.jmhsrobotics.frc2025.subsystems.elevator.Elevator;
import org.jmhsrobotics.frc2025.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.vision.Vision;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class TeleopCoralScoreSequence extends SequentialCommandGroup {
  public TeleopCoralScoreSequence(
      Drive drive,
      Elevator elevator,
      Wrist wrist,
      Intake intake,
      Indexer indexer,
      Vision vision,
      LED led) {
    boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    addCommands(
        // First sequential group only runs if at top and blue alliance, or at bottom and red
        // alliance
        new SequentialCommandGroup(
                new AutoIntakeCoral(drive, wrist, elevator, intake, indexer, led, false)
                    .onlyIf(() -> intake.getMode() == 2),
                new AutoScoreCoral(
                    drive,
                    elevator,
                    wrist,
                    intake,
                    indexer,
                    vision,
                    led,
                    true,
                    20,
                    Constants.ElevatorConstants.kLevel3Meters,
                    Constants.WristConstants.kLevel3Degrees),
                new AutoIntakeCoral(drive, wrist, elevator, intake, indexer, led, false),
                new AutoScoreCoral(
                    drive,
                    elevator,
                    wrist,
                    intake,
                    indexer,
                    vision,
                    led,
                    false,
                    20,
                    Constants.ElevatorConstants.kLevel3Meters,
                    Constants.WristConstants.kLevel3Degrees),
                new AutoIntakeCoral(drive, wrist, elevator, intake, indexer, led, false),
                new AutoScoreCoral(
                    drive,
                    elevator,
                    wrist,
                    intake,
                    indexer,
                    vision,
                    led,
                    true,
                    20,
                    Constants.ElevatorConstants.kLevel2Meters,
                    Constants.WristConstants.kLevel2Degrees),
                new AutoIntakeCoral(drive, wrist, elevator, intake, indexer, led, false),
                new AutoScoreCoral(
                    drive,
                    elevator,
                    wrist,
                    intake,
                    indexer,
                    vision,
                    led,
                    false,
                    20,
                    Constants.ElevatorConstants.kLevel2Meters,
                    Constants.WristConstants.kLevel2Degrees))
            .onlyIf(
                () ->
                    (drive.getPose().getY() > 4 && isBlueAlliance)
                        || (drive.getPose().getY() < 4 && !isBlueAlliance)),

        // runs if at bottom and blue alliance, or top and red alliance
        new SequentialCommandGroup(
                new AutoIntakeCoral(drive, wrist, elevator, intake, indexer, led, false)
                    .onlyIf(() -> intake.getMode() == 2),
                new AutoScoreCoral(
                    drive,
                    elevator,
                    wrist,
                    intake,
                    indexer,
                    vision,
                    led,
                    false,
                    22,
                    Constants.ElevatorConstants.kLevel3Meters,
                    Constants.WristConstants.kLevel3Degrees),
                new AutoIntakeCoral(drive, wrist, elevator, intake, indexer, led, false),
                new AutoScoreCoral(
                    drive,
                    elevator,
                    wrist,
                    intake,
                    indexer,
                    vision,
                    led,
                    true,
                    22,
                    Constants.ElevatorConstants.kLevel3Meters,
                    Constants.WristConstants.kLevel3Degrees),
                new AutoIntakeCoral(drive, wrist, elevator, intake, indexer, led, false),
                new AutoScoreCoral(
                    drive,
                    elevator,
                    wrist,
                    intake,
                    indexer,
                    vision,
                    led,
                    false,
                    22,
                    Constants.ElevatorConstants.kLevel2Meters,
                    Constants.WristConstants.kLevel2Degrees),
                new AutoIntakeCoral(drive, wrist, elevator, intake, indexer, led, false),
                new AutoScoreCoral(
                    drive,
                    elevator,
                    wrist,
                    intake,
                    indexer,
                    vision,
                    led,
                    true,
                    22,
                    Constants.ElevatorConstants.kLevel2Meters,
                    Constants.WristConstants.kLevel2Degrees))
            .onlyIf(
                () ->
                    (drive.getPose().getY() < 4 && isBlueAlliance)
                        || (drive.getPose().getY() > 4 && !isBlueAlliance)));
  }
}
