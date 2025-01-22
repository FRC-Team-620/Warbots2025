package org.jmhsrobotics.frc2025.controlBoard;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AltControlMode implements ControlBoard {
  XboxController driver = new XboxController(0);

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
    return new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  }
}
