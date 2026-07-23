package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import org.yaml.snakeyaml.Yaml;

public final class ConfigLoader {
  private static final String RELATIVE_EXTERNAL_PATH = "FIRST/teamcode/config.yaml";
  private static final String CLASSPATH_RESOURCE =
      "/org/firstinspires/ftc/teamcode/robot/config/config.yaml";

  private static Map<String, Object> config = new HashMap<>();

  static {
    reload();
  }

  /**
   * Reloads configuration from the highest-priority available source.
   *
   * <p>Priority: external ADB-pushed file → classpath resource. Call this at the top of {@code
   * OpMode.init()} after pushing a new YAML via ADB.
   */
  @SuppressWarnings("unchecked")
  public static synchronized void reload() {
    java.io.File externalFile = null;
    try {
      java.io.File storageDir = android.os.Environment.getExternalStorageDirectory();
      if (storageDir != null) {
        externalFile = new java.io.File(storageDir, RELATIVE_EXTERNAL_PATH);
      }
    } catch (Throwable ignored) {
      // android.os.Environment is unmocked in pure JVM unit tests
    }
    try {
      InputStream inputStream;
      if (externalFile != null && externalFile.exists() && externalFile.canRead()) {
        inputStream = new java.io.FileInputStream(externalFile);
      } else {
        inputStream = ConfigLoader.class.getResourceAsStream(CLASSPATH_RESOURCE);
      }

      if (inputStream == null) {
        throw new RuntimeException("Could not find config.yaml on the classpath");
      }
      try (InputStream is = inputStream) {
        Yaml yaml = new Yaml();
        Map<String, Object> loaded = yaml.load(is);
        config = loaded != null ? loaded : new HashMap<>();
      }
    } catch (Exception e) {
      throw new RuntimeException("Failed to load config.yaml", e);
    }
  }

  private static boolean isMirrorSignal(Object val) {
    if (val instanceof String) {
      String s = ((String) val).trim();
      return s.equalsIgnoreCase("m") || s.equalsIgnoreCase("mirror");
    }
    return false;
  }

  private static String getMirrorKey(String key) {
    if (key == null || key.isEmpty()) return key;
    String lower = key.toLowerCase();
    for (String[] pair : new String[][]{{"red", "blue"}, {"blue", "red"}}) {
      String alliance = pair[0], mirror = pair[1];
      String[][] variants = new String[][]{
        {alliance + "_", mirror + "_"},
        {"_" + alliance, "_" + mirror},
        {alliance.toUpperCase() + "_", mirror.toUpperCase() + "_"},
        {"_" + alliance.toUpperCase(), "_" + mirror.toUpperCase()}
      };
      for (String[] var : variants) {
        if (lower.contains(var[0])) {
          return lower.replace(var[0], var[1]);
        }
      }
      if (lower.equals(alliance)) return mirror;
    }
    return key;
  }

  private static String getMirrorPath(String path) {
    if (path == null || path.isEmpty()) return path;
    String[] segments = path.split("\\.");

    // 1. Check path segment replacement (e.g. auto_poses.normal.red.start -> auto_poses.normal.blue.start)
    for (String[] pair : new String[][]{{"red", "blue"}, {"blue", "red"}, {"RED", "BLUE"}, {"BLUE", "RED"}}) {
      String a = pair[0], m = pair[1];
      boolean matched = false;
      String[] mirrorSegs = new String[segments.length];
      for (int i = 0; i < segments.length; i++) {
        if (segments[i].equals(a)) {
          mirrorSegs[i] = m;
          matched = true;
        } else {
          mirrorSegs[i] = segments[i];
        }
      }
      if (matched) {
        return String.join(".", mirrorSegs);
      }
    }

    // 2. Leaf key replacement (e.g. red_base_power -> blue_base_power)
    String leaf = segments[segments.length - 1];
    String mirrorLeaf = getMirrorKey(leaf);
    if (!mirrorLeaf.equals(leaf)) {
      segments[segments.length - 1] = mirrorLeaf;
      return String.join(".", segments);
    }

    return path;
  }

  private static Object rawLoad(String path) {
    String[] parts = path.split("\\.");
    Object current = config;
    for (String part : parts) {
      if (current instanceof Map) {
        current = ((Map<String, Object>) current).get(part);
      } else {
        return null;
      }
    }
    return current;
  }

  private static Object resolveMirrorValue(String currentPath, Class<?> targetType) {
    String mirrorPath = getMirrorPath(currentPath);
    if (mirrorPath == null || mirrorPath.equals(currentPath)) {
      return null;
    }
    Object baseVal = rawLoad(mirrorPath);
    if (baseVal == null || isMirrorSignal(baseVal)) {
      return null;
    }
    if (targetType == Pose.class || (baseVal instanceof List && ((List<?>) baseVal).size() == 3)) {
      Pose basePose = (Pose) coerce(baseVal, Pose.class);
      if (basePose != null) {
        return basePose.mirror();
      }
    }
    return coerce(baseVal, targetType);
  }

  @SuppressWarnings("unchecked")
  public static <T> T load(Class<T> type, String path) {
    try {
      String[] parts = path.split("\\.");
      Object current = config;
      for (String part : parts) {
        if (current instanceof Map) {
          current = ((Map<String, Object>) current).get(part);
        } else {
          current = null;
          break;
        }
      }

      if (current instanceof Map) {
        // Nested configuration object: bind each public field from the map.
        T instance = type.getDeclaredConstructor().newInstance();
        Map<String, Object> map = (Map<String, Object>) current;
        for (Field field : type.getFields()) {
          Object val = map.get(field.getName());
          if (val == null) {
            val = map.get(toSnakeCase(field.getName()));
          }
          if (val != null) {
            field.setAccessible(true);
            if (isMirrorSignal(val)) {
              Object resolved = resolveMirrorValue(path + "." + toSnakeCase(field.getName()), field.getType());
              if (resolved != null) {
                field.set(instance, resolved);
              }
            } else {
              field.set(instance, coerce(val, field.getType()));
            }
          }
        }
        return instance;
      }

      if (current != null) {
        // Leaf node: a scalar, pose, array, or "m" mirror signal.
        if (isMirrorSignal(current)) {
          Object resolved = resolveMirrorValue(path, type);
          if (resolved != null) {
            return (T) resolved;
          }
        }
        return (T) coerce(current, type);
      }

      return defaultInstance(type);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load configuration into " + type.getName(), e);
    }
  }

  private static <T> T defaultInstance(Class<T> type) {
    try {
      return type.getDeclaredConstructor().newInstance();
    } catch (Exception e) {
      return null;
    }
  }

  /**
   * Loads a configuration class by merging multiple YAML paths. Fields are populated from each
   * path in order; later paths fill in fields not set by earlier ones (first path wins).
   */
  @SuppressWarnings("unchecked")
  public static <T> T loadMerged(Class<T> type, String... paths) {
    try {
      T instance = type.getDeclaredConstructor().newInstance();
      // The generator emits a separate "*Opposite*" config class whose shared fields come from the
      // opposite_-prefixed YAML keys (e.g. opposite_launch_power). Both the normal and opposite
      // classes use the same field names ("launchPower"), so for the opposite variant we must
      // prefer the opposite_ key — otherwise the un-prefixed normal key shadows it.
      boolean preferOpposite = type.getSimpleName().toLowerCase().contains("opposite");
      for (String path : paths) {
        String[] parts = path.split("\\.");
        Object current = config;
        for (String part : parts) {
          if (current instanceof Map) {
            current = ((Map<String, Object>) current).get(part);
          } else {
            current = null;
            break;
          }
        }
        if (!(current instanceof Map)) continue;
        Map<String, Object> map = (Map<String, Object>) current;
        for (Field field : type.getFields()) {
          // Don't overwrite a field already populated by an earlier path
          field.setAccessible(true);
          if (field.get(instance) != null && !field.getType().isPrimitive()) continue;
          Object val = map.get(field.getName());
          if (val == null) {
            String snakeName = toSnakeCase(field.getName());
            String oppositeKey = "opposite_" + snakeName;
            if (preferOpposite && map.containsKey(oppositeKey)) {
              val = map.get(oppositeKey);
            } else {
              val = map.get(snakeName);
              if (val == null) {
                val = map.get(oppositeKey);
              }
            }
          }
          if (val != null) {
            if (isMirrorSignal(val)) {
              String snakeName = toSnakeCase(field.getName());
              Object resolved = resolveMirrorValue(path + "." + snakeName, field.getType());
              if (resolved == null) {
                String mirrorKey = getMirrorKey(snakeName);
                if (map.containsKey(mirrorKey)) {
                  Object rawMirrorVal = map.get(mirrorKey);
                  if (rawMirrorVal != null && !isMirrorSignal(rawMirrorVal)) {
                    if (field.getType() == Pose.class || (rawMirrorVal instanceof List && ((List<?>) rawMirrorVal).size() == 3)) {
                      Pose basePose = (Pose) coerce(rawMirrorVal, Pose.class);
                      resolved = basePose != null ? basePose.mirror() : null;
                    } else {
                      resolved = coerce(rawMirrorVal, field.getType());
                    }
                  }
                }
              }
              if (resolved != null) {
                field.set(instance, resolved);
              }
            } else {
              field.set(instance, coerce(val, field.getType()));
            }
          }
        }
      }
      return instance;
    } catch (Exception e) {
      throw new RuntimeException("Failed to loadMerged configuration into " + type.getName(), e);
    }
  }

  private static String toSnakeCase(String name) {
    return name.replaceAll("([a-z0-9])([A-Z])", "$1_$2").toLowerCase();
  }

  @SuppressWarnings("unchecked")
  private static Object coerce(Object value, Class<?> type) {
    if (type.isInstance(value)) {
      return value;
    }
    if (type == Pose.class && value instanceof List) {
      List<Number> list = (List<Number>) value;
      if (list.size() >= 3) {
        double x = list.get(0).doubleValue();
        double y = list.get(1).doubleValue();
        double heading = list.get(2).doubleValue();
        return new Pose(x, y, Math.toRadians(heading));
      }
    }
    if (type == Pose.class && value instanceof Map) {
      Map<String, Object> m = (Map<String, Object>) value;
      double x = ((Number) Objects.requireNonNull(m.get("x"))).doubleValue();
      double y = ((Number) Objects.requireNonNull(m.get("y"))).doubleValue();
      double heading = ((Number) Objects.requireNonNull(m.get("heading"))).doubleValue();
      return new Pose(x, y, Math.toRadians(heading));
    }
    if ((type.getName().equals("com.qualcomm.robotcore.hardware.PIDFCoefficients")
            || type.getName().equals("com.pedropathing.control.PIDFCoefficients"))
        && value instanceof Map) {
      Map<String, Object> m = (Map<String, Object>) value;
      double p = m.containsKey("p") ? ((Number) Objects.requireNonNull(m.get("p"))).doubleValue() : 0;
      double i = m.containsKey("i") ? ((Number) Objects.requireNonNull(m.get("i"))).doubleValue() : 0;
      double d = m.containsKey("d") ? ((Number) Objects.requireNonNull(m.get("d"))).doubleValue() : 0;
      double f = m.containsKey("f") ? ((Number) Objects.requireNonNull(m.get("f"))).doubleValue() : 0;
      if (type.getName().equals("com.qualcomm.robotcore.hardware.PIDFCoefficients")) {
        try {
          return Class.forName("com.qualcomm.robotcore.hardware.PIDFCoefficients")
              .getConstructor(double.class, double.class, double.class, double.class)
              .newInstance(p, i, d, f);
        } catch (Exception e) {
          throw new RuntimeException("Failed to instantiate PIDFCoefficients", e);
        }
      } else {
        return new com.pedropathing.control.PIDFCoefficients(p, i, d, f);
      }
    }
    if (type == double.class || type == Double.class) {
      return ((Number) value).doubleValue();
    }
    if (type == int.class || type == Integer.class) {
      return ((Number) value).intValue();
    }
    if (type == long.class || type == Long.class) {
      return ((Number) value).longValue();
    }
    if (type == float.class || type == Float.class) {
      return ((Number) value).floatValue();
    }
    if (type == boolean.class || type == Boolean.class) {
      return (Boolean) value;
    }
    if (type == String.class) {
      return value.toString();
    }
    // Recursively handle nested configuration objects (e.g. Turret$Orientation)
    if (value instanceof Map && !type.isPrimitive()) {
      try {
        @SuppressWarnings("unchecked")
        Map<String, Object> nestedMap = (Map<String, Object>) value;
        Object nestedInstance = type.getDeclaredConstructor().newInstance();
        for (Field field : type.getFields()) {
          Object fieldVal = nestedMap.get(field.getName());
          if (fieldVal == null) {
            String snakeName = toSnakeCase(field.getName());
            fieldVal = nestedMap.get(snakeName);
          }
          if (fieldVal != null) {
            field.setAccessible(true);
            field.set(nestedInstance, coerce(fieldVal, field.getType()));
          }
        }
        return nestedInstance;
      } catch (Exception e) {
        // Fall through and return the raw map as a last resort
      }
    }
    return value;
  }
}
