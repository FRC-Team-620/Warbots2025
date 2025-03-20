package org.jmhsrobotics.frc2025.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.XboxController;
import java.util.HashMap;
import java.util.Map;

public class ControllerMonitor {
  private static Map<XboxController, Alert[]> controllermap =
      new HashMap<XboxController, Alert[]>();

  public static void checkController() {
    for (XboxController control : controllermap.keySet()) {
      if (control.isConnected()
          && control.getType()
              != null) { // Weird Work around, controler.isConnected does not seem to work correctly
        // when reconnecting. See
        // https://github.com/wpilibsuite/allwpilib/issues/7700
        controllermap.get(control)[0].set(!(control.getButtonCount() == 10));
        controllermap.get(control)[1].set(false);
      } else {

        controllermap.get(control)[0].set(false);
        controllermap.get(control)[1].set(true);
      }
    }
  }

  public static void addController(XboxController c, String a) {
    controllermap.put(
        c,
        new Alert[] {
          new Alert("Wrong Controller Mode: " + a, Alert.AlertType.kError),
          new Alert("Controller Disconnected: " + a, Alert.AlertType.kWarning)
        });
  }
}
