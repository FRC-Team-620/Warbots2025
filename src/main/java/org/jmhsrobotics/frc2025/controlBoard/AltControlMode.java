package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AltControlMode implements ControlBoard {
  CommandXboxController driver = new CommandXboxController(0);

  private Trigger nop =
      new Trigger(
          () -> {
            return false;
          });

  @Override
  public double rotation() {
    return driver.getRightX();
  }

  @Override
  public double translationX() {
    return calculateTotalTranslation(true);
  }

  @Override
  public double translationY() {
    return calculateTotalTranslation(false);
  }

  // normalizes the X and Y translations and puts them in a vector
  private double calculateTotalTranslation(boolean isX) {
    double driveX = driver.getLeftX();
    double driveY = driver.getLeftY();
    if (driveX == 0 && driveY == 0) {
      return 0;
    }
    Vector<N2> translationVector = VecBuilder.fill(driveX, driveY);
    translationVector = translationVector.unit();

    // multiply the normalized translation by the amount that the right trigger is pressed
    // if(isX){
    //     return translationVector.get(0) * driver.getRightTriggerAxis();
    // }
    return translationVector.get(isX ? 0 : 1) * driver.getRightTriggerAxis();
  }

  @Override
  public Trigger resetForward() {
    // return new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    return driver.rightBumper();
  }

  @Override
  public Trigger turboMode() {
    return nop;
  }

  @Override
  public Trigger upExample() {
    return driver.x();
  }

  @Override
  public Trigger downExample() {
    return driver.a();
  }

  public Trigger intakeCoral() {
    return nop;
  }

  public Trigger extakeCoral() {
    return nop;
  }

  public Trigger placeCoralL1() {
    return nop;
  }

  public Trigger placeCoralL2() {
    return nop;
  }

  public Trigger placeCoralL3() {
    return nop;
  }

  public Trigger placeCoralL4() {
    return nop;
  }

  public Trigger removeAlgaeL23() {
    return nop;
  }

  public Trigger removeAlgaeL34() {
    return nop;
  }

  public Trigger scoreProcessor() {
    return nop;
  }

  public Trigger scoreBarge() {
    return nop;
  }

  public Trigger climbUp() {
    return nop;
  }

  public Trigger climbDown() {
    return nop;
  }

  public Trigger indexerUp() {
    return nop;
  }

  public Trigger indexerDown() {
    return nop;
  }
}
