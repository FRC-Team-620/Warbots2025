package org.jmhsrobotics.frc2025.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.XboxController;
import java.util.HashMap;
import java.util.Map;

public class ControllerMonitor {
  private static Map<XboxController, Alert> controllermap = new HashMap<XboxController, Alert>();

  public static void checkController() {
    for (XboxController control : controllermap.keySet()) {
      controllermap.get(control).set((control.getAxisCount() == 6));
    }
  }

  public static void addController(XboxController c, String a) {

    controllermap.put(c, new Alert("Wrong Controller Mode: " + a, Alert.AlertType.kError));
  }
}
