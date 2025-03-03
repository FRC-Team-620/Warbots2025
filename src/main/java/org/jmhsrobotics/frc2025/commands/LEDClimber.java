package org.jmhsrobotics.frc2025.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2025.Constants;
import org.jmhsrobotics.frc2025.subsystems.climber.Climber;
import org.jmhsrobotics.frc2025.subsystems.led.LED;

public class LEDClimber extends Command {
  private Climber climber;
  private LED led;

  private final LEDPattern topPattern = LEDPattern.solid(Color.kOrange);
  private final LEDPattern progressPattern;
  private LEDPattern bottomPattern = LEDPattern.solid(Color.kWhite);

  // LEDPattern basePattern = gradient(Color.kRed, Color.kBlue);
  // LEDPattern progressPattern =
  //   basePattern.mask(progressMaskLayer(() -> elevator.getHeight() / elevator.maxHeight());

  public LEDClimber(Climber climber, LED led) {
    this.climber = climber;
    this.led = led;
    progressPattern =
        LEDPattern.progressMaskLayer(
            () ->
                (climber.getAngleDegrees() - Constants.ClimberConstants.kSoftLimitTopDegrees)
                    / Constants.ClimberConstants.kSoftLimitBottomDegrees);
    addRequirements(led);
  }

  @Override
  public void initialize() {
    if (climber.getAngleDegrees() < Constants.ClimberConstants.kSoftLimitTopDegrees + 1)
      this.led.setPattern(topPattern);
    else this.led.setPattern(progressPattern);
  }

  @Override
  public void execute() {
    if (climber.getAngleDegrees() < Constants.ClimberConstants.kSoftLimitTopDegrees + 1)
      this.led.setPattern(topPattern);
    else this.led.setPattern(progressPattern);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
