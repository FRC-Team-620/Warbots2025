package org.jmhsrobotics.frc2025.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.intake.Intake;
import org.jmhsrobotics.frc2025.subsystems.led.LED;
import org.jmhsrobotics.frc2025.subsystems.wrist.Wrist;

public class IntakeFromIndexer extends Command {
  private Wrist wrist;
  private Intake intake;
  private LED led;

  /// private boolean coralIntaked;

  private LEDPattern blinkPattern = LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.1));

  public IntakeFromIndexer(Wrist wrist, Intake intake, LED led) {
    this.wrist = wrist;
    this.intake = intake;
    this.led = led;

    addRequirements(wrist, intake, led);
  }

  @Override
  public void initialize() {
    intake.set(0);
    wrist.setSetpoint(Constants.WristConstants.kRotationIntakeCoralDegrees);

    // EVERYTHING THAT HAS 3 SLASHES CAN BE UNCOMMENTED TO MAKE THE FIRST HALF OF THE PLACEMENT FIX
    // COMMAND PART OF THE INTAKE COMMAND
    // SHOULD BE DONE WE ARE ABLE TO FIND A WAY TO GET THE ACTIVE INDEXER WORKING AND ABLE TO DETECT
    // CORAL

    /// this.coralIntaked = false;
    led.setPattern(blinkPattern);
  }

  @Override
  public void execute() {
    led.setPattern(blinkPattern);
    /// if (intake.isCoralInIntake()) {
    ///   intake.set(Constants.IntakeConstants.kCoralDefaultCommandSpeed * 0.8);
    ///   coralIntaked = true;
    /// } else intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
    intake.set(Constants.IntakeConstants.kCoralIntakeIndexerSpeedDutyCycle);
  }

  @Override
  public boolean isFinished() {
    /// return coralIntaked && !intake.isCoralInIntake();
    return intake.isCoralInIntake();
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    /// wrist.setSetpoint(Constants.WristConstants.kSafeAngleDegrees);
    System.out.println("Intake From Indexer Complete: " + interrupted);
  }
}
