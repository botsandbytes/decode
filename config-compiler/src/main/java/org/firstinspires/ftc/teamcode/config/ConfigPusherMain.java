package org.firstinspires.ftc.teamcode.config;

import java.io.BufferedReader;
import java.io.File;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

/**
 * Pure Java utility for pushing config.yaml to the robot via ADB.
 * Eliminates external Python interpreter dependency.
 */
public class ConfigPusherMain {

  public static void main(String[] args) {
    boolean reset = false;
    String projectDir = ".";

    for (String arg : args) {
      if ("--reset".equals(arg)) {
        reset = true;
      } else if (!arg.startsWith("-")) {
        projectDir = arg;
      }
    }

    String device = getDevice();
    if (device == null) {
      System.err.println("ERROR: No ADB device found. Is the Control Hub connected and ADB enabled?");
      System.exit(1);
    }

    if (reset) {
      resetConfig(device);
    } else {
      pushConfig(device, projectDir);
    }
  }

  private static String getDevice() {
    try {
      Process p = new ProcessBuilder("adb", "devices").start();
      BufferedReader reader = new BufferedReader(new InputStreamReader(p.getInputStream()));
      String line;
      List<String> devices = new ArrayList<>();
      while ((line = reader.readLine()) != null) {
        line = line.trim();
        if (!line.isEmpty() && !line.contains("List of devices") && line.contains("device") && !line.contains("offline")) {
          devices.add(line.split("\\s+")[0]);
        }
      }
      p.waitFor();
      if (devices.isEmpty()) return null;
      return devices.get(0);
    } catch (Exception e) {
      return null;
    }
  }

  private static void pushConfig(String device, String projectDir) {
    System.out.println("Pushing config.yaml → robot (" + device + ")");
    File localYaml = new File(projectDir, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml");
    if (!localYaml.exists()) {
      System.err.println("ERROR: Could not find config.yaml at " + localYaml.getAbsolutePath());
      System.exit(1);
    }

    exec("adb", "-s", device, "shell", "mkdir", "-p", "/sdcard/FIRST/teamcode", "/sdcard/FIRST");
    exec("adb", "-s", device, "push", localYaml.getAbsolutePath(), "/sdcard/FIRST/teamcode/config.yaml.tmp");
    exec("adb", "-s", device, "shell", "mv", "/sdcard/FIRST/teamcode/config.yaml.tmp", "/sdcard/FIRST/teamcode/config.yaml");
    exec("adb", "-s", device, "push", localYaml.getAbsolutePath(), "/sdcard/FIRST/config.yaml.tmp");
    exec("adb", "-s", device, "shell", "mv", "/sdcard/FIRST/config.yaml.tmp", "/sdcard/FIRST/config.yaml");
    System.out.println("✓ Pushed config.yaml atomically to robot (/sdcard/FIRST/teamcode/config.yaml)");
    System.out.println("  Re-init your OpMode on the robot to pick up the new values.");
  }

  private static void resetConfig(String device) {
    System.out.println("Removing config override from robot (" + device + ")");
    exec("adb", "-s", device, "shell", "rm", "-f", "/sdcard/FIRST/teamcode/config.yaml", "/sdcard/FIRST/config.yaml");
    System.out.println("✓ Removed override files — robot will use bundled config on next init.");
  }

  private static void exec(String... command) {
    try {
      Process p = new ProcessBuilder(command).start();
      p.waitFor();
    } catch (Exception ignored) {}
  }
}
