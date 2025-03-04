package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.climber.Climber;
import org.jmhsrobotics.frc2025.subsystems.led.LED;

public class ClimberMove extends Command {
  private Climber climber;
  private double climberSpeedDutyCycle;
  private LED led;

  private final LEDPattern topPattern = LEDPattern.solid(Color.kRed);
  private final LEDPattern progressPattern;
  private final LEDPattern bottomPattern = LEDPattern.solid(Color.kGreen);

  public ClimberMove(Climber climber, LED led, double climberSpeedDutyCycle) {
    this.climber = climber;
    this.led = led;
    this.climberSpeedDutyCycle = climberSpeedDutyCycle;

    // creates the progress bar LED pattern
    progressPattern =
        LEDPattern.progressMaskLayer(
            () ->
                (climber.getAngleDegrees() - Constants.ClimberConstants.kSoftLimitTopDegrees)
                    / Constants.ClimberConstants.kSoftLimitBottomDegrees);

    addRequirements(climber, led);
  }

  @Override
  public void initialize() {
    this.climber.setSpeedDutyCycle(climberSpeedDutyCycle);

    // Sets LED pattern based on climber position
    if (climber.getAngleDegrees() < Constants.ClimberConstants.kSoftLimitTopDegrees + 1)
      this.led.setPattern(topPattern);
    else if (climber.getAngleDegrees() > Constants.ClimberConstants.kSoftLimitBottomDegrees - 10)
      this.led.setPattern(bottomPattern);
    else this.led.setPattern(progressPattern);
  }

  @Override
  public void execute() {
    this.climber.setSpeedDutyCycle(climberSpeedDutyCycle);

    // Sets LED pattern based on climber position
    if (climber.getAngleDegrees() < Constants.ClimberConstants.kSoftLimitTopDegrees + 1)
      this.led.setPattern(topPattern);
    else if (climber.getAngleDegrees() > Constants.ClimberConstants.kSoftLimitBottomDegrees - 1)
      this.led.setPattern(bottomPattern);
    else this.led.setPattern(progressPattern);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.climber.setSpeedDutyCycle(0);
  }
}
