package org.firstinspires.ftc.teamcode.config;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.TreeSet;
import org.yaml.snakeyaml.Yaml;

public class ConfigGeneratorMain {

  public static void main(String[] args) {
    if (args.length < 1) {
      System.err.println("Usage: ConfigGeneratorMain <rootDir> [mode: generate|check]");
      System.exit(1);
    }

    String rootDir = args[0];
    String mode = args.length >= 2 ? args[1] : "generate";

    File configYaml =
        new File(
            rootDir,
            "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml");
    File docsYaml =
        new File(
            rootDir,
            "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config-docs.yaml");
    File javaOut =
        new File(
            rootDir,
            "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.java");
    File schemaOut =
        new File(
            rootDir,
            "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config-schema.json");

    if (!configYaml.exists()) {
      System.err.println("Error: config.yaml not found at " + configYaml.getAbsolutePath());
      System.exit(1);
    }

    Yaml yaml = new Yaml();
    Map<String, Object> configData;
    Map<String, Object> docsData;

    try (FileInputStream inConfig = new FileInputStream(configYaml)) {
      configData = yaml.load(inConfig);
    } catch (Exception e) {
      System.err.println("Error reading config.yaml: " + e.getMessage());
      System.exit(1);
      return;
    }

    try (FileInputStream inDocs = new FileInputStream(docsYaml)) {
      docsData = docsYaml.exists() ? yaml.load(inDocs) : Collections.emptyMap();
    } catch (Exception e) {
      docsData = Collections.emptyMap();
    }

    if (configData == null) {
      configData = Collections.emptyMap();
    }
    if (docsData == null) {
      docsData = Collections.emptyMap();
    }

    Map<String, DocInfo> docsMap = parseDocs(docsData);
    Set<String> suppressedKeys = parseSuppressions(docsData);
    List<String> flatKeys = flattenKeys(configData, "");

    // Perform validation checks
    List<String> symmetryErrors = new ArrayList<>();
    symmetryErrors.addAll(checkAllianceSymmetry(flatKeys, suppressedKeys));
    symmetryErrors.addAll(checkPathAllianceSymmetry(flatKeys, suppressedKeys));
    List<String> fuzzyWarnings = checkFuzzyDuplicates(flatKeys, suppressedKeys);

    if (!fuzzyWarnings.isEmpty()) {
      System.out.println("CONFIG CHECK WARNINGS (not build-blocking):");
      for (String w : fuzzyWarnings) {
        System.out.println(w);
      }
      System.out.println();
    }

    if ("check".equalsIgnoreCase(mode)) {
      if (!symmetryErrors.isEmpty()) {
        System.err.println("CONFIG CHECK ERRORS:");
        for (String err : symmetryErrors) {
          System.err.println(err);
        }
        System.exit(1);
      } else {
        System.out.println("Config key check passed.");
        System.exit(0);
      }
      return;
    }

    // "generate" mode
    if (!symmetryErrors.isEmpty()) {
      System.err.println("CONFIG CHECK ERRORS:");
      for (String err : symmetryErrors) {
        System.err.println(err);
      }
      System.exit(1);
    }

    String javaCode = generateJavaClass(configData, docsMap);
    String schemaJson = generateSchemaJson(configData, docsMap);

    boolean javaChanged = writeIfChanged(javaOut, javaCode);
    boolean schemaChanged = writeIfChanged(schemaOut, schemaJson);

    if (javaChanged) {
      System.out.println("Generated " + javaOut.getAbsolutePath());
    } else {
      System.out.println("Up-to-date: " + javaOut.getAbsolutePath());
    }

    if (schemaChanged) {
      System.out.println("Generated " + schemaOut.getAbsolutePath());
    } else {
      System.out.println("Up-to-date: " + schemaOut.getAbsolutePath());
    }
  }

  // --- Helper Classes & Data Structures ---

  private static class DocInfo {
    String desc = "";
    Double min = null;
    Double max = null;
  }

  @SuppressWarnings("unchecked")
  private static Map<String, DocInfo> parseDocs(Map<String, Object> docsData) {
    Map<String, DocInfo> map = new HashMap<>();
    for (Map.Entry<String, Object> entry : docsData.entrySet()) {
      DocInfo info = new DocInfo();
      if (entry.getValue() instanceof Map) {
        Map<String, Object> sub = (Map<String, Object>) entry.getValue();
        Object d = sub.get("desc");
        if (d != null) info.desc = d.toString();
        if (sub.get("min") instanceof Number) info.min = ((Number) Objects.requireNonNull(sub.get("min"))).doubleValue();
        if (sub.get("max") instanceof Number) info.max = ((Number) Objects.requireNonNull(sub.get("max"))).doubleValue();
      } else if (entry.getValue() != null) {
        info.desc = entry.getValue().toString();
      }
      map.put(entry.getKey(), info);
    }
    return map;
  }

  @SuppressWarnings("unchecked")
  private static Set<String> parseSuppressions(Map<String, Object> docsData) {
    Set<String> suppressed = new HashSet<>();
    for (Map.Entry<String, Object> entry : docsData.entrySet()) {
      if (entry.getValue() instanceof Map) {
        Map<String, Object> sub = (Map<String, Object>) entry.getValue();
        Object val = sub.get("suppress_similarity_check");
        if (val != null && ("true".equalsIgnoreCase(val.toString()) || Boolean.TRUE.equals(val))) {
          suppressed.add(entry.getKey());
        }
      }
    }
    return suppressed;
  }

  @SuppressWarnings("unchecked")
  private static List<String> flattenKeys(Map<String, Object> map, String prefix) {
    List<String> keys = new ArrayList<>();
    for (Map.Entry<String, Object> entry : map.entrySet()) {
      String full = prefix.isEmpty() ? entry.getKey() : prefix + "." + entry.getKey();
      if (entry.getValue() instanceof Map) {
        keys.addAll(flattenKeys((Map<String, Object>) entry.getValue(), full));
      } else {
        keys.add(full);
      }
    }
    return keys;
  }

  // --- Symmetry & Similarity Validation ---

  private static List<String> checkAllianceSymmetry(List<String> flatKeys, Set<String> suppressed) {
    List<String> errors = new ArrayList<>();
    Map<String, String> leafToPath = new HashMap<>();
    for (String k : flatKeys) {
      String[] parts = k.split("\\.");
      leafToPath.put(parts[parts.length - 1], k);
    }

    String[][] pairs = {{"red", "blue"}, {"blue", "red"}};
    for (Map.Entry<String, String> entry : leafToPath.entrySet()) {
      String leaf = entry.getKey();
      String path = entry.getValue();
      if (suppressed.contains(leaf)) continue;
      String leafLower = leaf.toLowerCase();

      for (String[] pair : pairs) {
        String alliance = pair[0];
        String mirror = pair[1];
        String[][] variants = {
          {alliance + "_", mirror + "_"},
          {"_" + alliance, "_" + mirror},
          {alliance.toUpperCase() + "_", mirror.toUpperCase() + "_"},
          {"_" + alliance.toUpperCase(), "_" + mirror.toUpperCase()}
        };

        for (String[] var : variants) {
          if (leafLower.contains(var[0])) {
            String expectedLeaf = leafLower.replace(var[0], var[1]);
            boolean mirrorExists = false;
            for (String k : flatKeys) {
              String[] kParts = k.split("\\.");
              if (kParts[kParts.length - 1].equalsIgnoreCase(expectedLeaf)) {
                mirrorExists = true;
                break;
              }
            }
            if (!mirrorExists && !suppressed.contains(expectedLeaf)) {
              errors.add(
                  "  Alliance asymmetry: '"
                      + leaf
                      + "' found (at "
                      + path
                      + ")\n"
                      + "    but no mirror '"
                      + expectedLeaf
                      + "' exists.\n"
                      + "    → Add the mirror key, or add suppress_similarity_check: true\n"
                      + "      under '"
                      + leaf
                      + "' in config-docs.yaml");
            }
          }
        }
      }
    }
    return errors;
  }

  private static List<String> checkPathAllianceSymmetry(
      List<String> flatKeys, Set<String> suppressed) {
    List<String> errors = new ArrayList<>();
    Set<String> keySet = new HashSet<>(flatKeys);

    for (String key : flatKeys) {
      String[] segments = key.split("\\.");
      String leaf = segments[segments.length - 1];
      if (suppressed.contains(leaf)) continue;

      String[][] pairs = {{"red", "blue"}, {"blue", "red"}};
      for (String[] pair : pairs) {
        String alliance = pair[0];
        String mirror = pair[1];

        boolean hasSegment = false;
        String[] mirrorSegments = new String[segments.length];
        for (int i = 0; i < segments.length; i++) {
          if (segments[i].equalsIgnoreCase(alliance)) {
            mirrorSegments[i] = mirror;
            hasSegment = true;
          } else {
            mirrorSegments[i] = segments[i];
          }
        }

        if (!hasSegment) continue;

        String mirrorKey = String.join(".", mirrorSegments);
        if (!keySet.contains(mirrorKey)) {
          errors.add(
              "  Alliance path asymmetry: '"
                  + key
                  + "'\n"
                  + "    but no mirror '"
                  + mirrorKey
                  + "' exists.\n"
                  + "    → Add the mirror key, or add suppress_similarity_check: true\n"
                  + "      under '"
                  + leaf
                  + "' in config-docs.yaml");
        }
      }
    }
    return errors;
  }

  private static List<String> checkFuzzyDuplicates(List<String> flatKeys, Set<String> suppressed) {
    List<String> warnings = new ArrayList<>();
    List<String> names = new ArrayList<>();
    for (String k : flatKeys) {
      String[] parts = k.split("\\.");
      names.add(parts[parts.length - 1]);
    }

    Set<String> seenPairs = new HashSet<>();
    for (int i = 0; i < names.size(); i++) {
      for (int j = i + 1; j < names.size(); j++) {
        String a = names.get(i);
        String b = names.get(j);
        String pairKey = a.compareTo(b) < 0 ? a + ":" + b : b + ":" + a;
        if (seenPairs.contains(pairKey)) continue;
        seenPairs.add(pairKey);

        if (suppressed.contains(a) || suppressed.contains(b)) continue;
        if (a.length() <= 2 || b.length() <= 2) continue;
        if (a.equalsIgnoreCase(b)) continue;
        if (a.replaceAll("\\d+", "").equalsIgnoreCase(b.replaceAll("\\d+", ""))) continue;
        if (normalize(a).equalsIgnoreCase(normalize(b))) continue;

        int dist = levenshtein(a.toLowerCase(), b.toLowerCase());
        if (dist > 0 && dist <= 2) {
          warnings.add(
              "  Fuzzy duplicate: '"
                  + a
                  + "' and '"
                  + b
                  + "' differ by only "
                  + dist
                  + " character(s).\n"
                  + "    → Confirm these are intentionally distinct, or suppress with\n"
                  + "      suppress_similarity_check: true in config-docs.yaml");
        }
      }
    }
    return warnings;
  }

  private static final List<Set<String>> STRUCTURAL_PAIRS =
      Arrays.asList(
          new HashSet<>(Arrays.asList("min", "max")),
          new HashSet<>(Arrays.asList("x", "y", "z")),
          new HashSet<>(Arrays.asList("red", "blue")),
          new HashSet<>(Arrays.asList("near", "far")),
          new HashSet<>(Arrays.asList("slow", "hard")),
          new HashSet<>(Arrays.asList("start", "end", "cp")),
          new HashSet<>(Arrays.asList("up", "down")),
          new HashSet<>(Arrays.asList("left", "right")),
          new HashSet<>(Arrays.asList("front", "back")),
          new HashSet<>(Arrays.asList("roll", "pitch", "yaw")),
          new HashSet<>(Arrays.asList("goal", "size")));

  private static String normalize(String name) {
    String[] parts = name.toLowerCase().split("[_\\s]");
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < parts.length; i++) {
      String part = parts[i];
      boolean replaced = false;
      for (int p = 0; p < STRUCTURAL_PAIRS.size(); p++) {
        if (STRUCTURAL_PAIRS.get(p).contains(part)) {
          sb.append("__PAIR").append(p).append("__");
          replaced = true;
          break;
        }
      }
      if (!replaced) sb.append(part);
      if (i < parts.length - 1) sb.append("_");
    }
    return sb.toString();
  }

  private static int levenshtein(String a, String b) {
    int m = a.length();
    int n = b.length();
    int[] dp = new int[n + 1];
    for (int j = 0; j <= n; j++) dp[j] = j;

    for (int i = 1; i <= m; i++) {
      int[] prev = dp.clone();
      dp[0] = i;
      for (int j = 1; j <= n; j++) {
        if (a.charAt(i - 1) == b.charAt(j - 1)) {
          dp[j] = prev[j - 1];
        } else {
          dp[j] = 1 + Math.min(prev[j], Math.min(dp[j - 1], prev[j - 1]));
        }
      }
    }
    return dp[n];
  }

  // --- Java Class Generation ---

  @SuppressWarnings("unchecked")
  private static String generateJavaClass(
      Map<String, Object> configData, Map<String, DocInfo> docs) {
    StringBuilder sb = new StringBuilder();
    sb.append("// AUTO-GENERATED FILE - DO NOT EDIT DIRECTLY\n");
    sb.append("package org.firstinspires.ftc.teamcode.robot.config;\n\n");
    sb.append("import com.pedropathing.geometry.Pose;\n");
    sb.append("import org.firstinspires.ftc.teamcode.config.ConfigLoader;\n");
    sb.append("import org.firstinspires.ftc.teamcode.records.Alliance;\n");
    sb.append("import org.firstinspires.ftc.teamcode.records.Field;\n");
    sb.append("import org.firstinspires.ftc.teamcode.records.MatchProfile;\n\n");


    sb.append("@SuppressWarnings(\"unused\")\n");
    sb.append("public final class config {\n");


    sb.append("  private config() {}\n\n");


    List<String> sections = new ArrayList<>();
    for (Map.Entry<String, Object> entry : configData.entrySet()) {
      if (entry.getValue() instanceof Map && !"auto_poses".equals(entry.getKey())) {
        sections.add(entry.getKey());
      }
    }

    List<LeafPath> leafPaths = new ArrayList<>();
    for (String s : sections) {
      collectLeafPaths((Map<String, Object>) Objects.requireNonNull(configData.get(s)), Collections.singletonList(s), leafPaths);
    }

    // Static section object fields
    for (String sectionKey : sections) {
      String className = toClassName(sectionKey);
      sb.append("  public static ").append(className).append(" ").append(sectionKey).append(";\n");
    }
    sb.append("\n");

    // Static uppercase aliases
    for (LeafPath lp : leafPaths) {
      String aliasName = String.join("_", lp.path).toUpperCase();
      String javaType = getJavaType(lp.value, lp.path.get(lp.path.size() - 1), lp.path, configData);
      sb.append("  public static ")
          .append(javaType)
          .append(" ")
          .append(aliasName)
          .append(";\n");
    }

    sb.append("\n  static {\n    reload();\n  }\n\n");
    sb.append("  public static synchronized void reload() {\n");
    sb.append("    ConfigLoader.reload();\n");
    for (String sectionKey : sections) {
      String className = toClassName(sectionKey);
      sb.append("    ")
          .append(sectionKey)
          .append(" = ConfigLoader.load(")
          .append(className)
          .append(".class, \"")
          .append(sectionKey)
          .append("\");\n");
    }
    for (LeafPath lp : leafPaths) {
      String javaPath = String.join(".", lp.path);
      String aliasName = String.join("_", lp.path).toUpperCase();
      sb.append("    ").append(aliasName).append(" = ").append(javaPath).append(";\n");
    }
    sb.append("  }\n\n");

    // Nested classes for sections
    for (Map.Entry<String, Object> entry : configData.entrySet()) {
      if (entry.getValue() instanceof Map && !"auto_poses".equals(entry.getKey())) {
        String className = toClassName(entry.getKey());
        generateNestedClass(sb, className, (Map<String, Object>) entry.getValue(), docs, "  ");
        sb.append("\n");
      }
    }

    // AutoPoses classes
    if (configData.containsKey("auto_poses")
        && configData.get("auto_poses") instanceof Map) {
      Map<String, Object> autoPoses = (Map<String, Object>) configData.get("auto_poses");
      Map<String, Object> autoGlobal =
          configData.get("auto") instanceof Map
              ? (Map<String, Object>) configData.get("auto")
              : Collections.emptyMap();

      Map<String, Object> normalGlobal = new LinkedHashMap<>();
      Map<String, Object> oppositeGlobal = new LinkedHashMap<>();
      for (Map.Entry<String, Object> e : Objects.requireNonNull(autoGlobal).entrySet()) {
        if (e.getKey().startsWith("opposite_")) {
          oppositeGlobal.put(e.getKey().substring("opposite_".length()), e.getValue());
        } else {
          normalGlobal.put(e.getKey(), e.getValue());
        }
      }

      if (Objects.requireNonNull(autoPoses).containsKey("normal") && autoPoses.get("normal") instanceof Map) {
        emitAllianceClass(
            sb, "NormalAuto", (Map<String, Object>) autoPoses.get("normal"), normalGlobal, docs);
      }
      if (autoPoses.containsKey("opposite") && autoPoses.get("opposite") instanceof Map) {
        emitAllianceClass(
            sb,
            "OppositeAuto",
            (Map<String, Object>) autoPoses.get("opposite"),
            oppositeGlobal,
            docs);
      }
    }

    // loadMatchProfile helper
    sb.append("  public static MatchProfile loadMatchProfile(Alliance alliance) {\n");
    sb.append("    String allianceStr = alliance == Alliance.RED ? \"red\" : \"blue\";\n");
    sb.append("    double goalX = Field.getGoalX(alliance);\n");
    sb.append("    double goalY = Field.getGoalY(alliance);\n");
    sb.append(
        "    Pose startPose = ConfigLoader.load(Pose.class, \"teleop.poses.\" + allianceStr +"
            + " \".start\");\n");
    sb.append(
        "    Pose drinkPose = ConfigLoader.load(Pose.class, \"teleop.poses.\" + allianceStr +"
            + " \".drink\");\n");
    sb.append(
        "    Pose parkPose = ConfigLoader.load(Pose.class, \"teleop.poses.\" + allianceStr +"
            + " \".park\");\n");
    sb.append("    Pose scorePose;\n");
    sb.append("    if (alliance == Alliance.RED) {\n");
    sb.append("      Pose redScore = ConfigLoader.load(Pose.class, \"teleop.poses.red.score\");\n");
    sb.append(
        "      scorePose = org.firstinspires.ftc.teamcode.robot.Turret.alignPose(redScore.getX(),"
            + " redScore.getY(), goalX + 2.5, goalY);\n");
    sb.append("    } else {\n");
    sb.append(
        "      scorePose = ConfigLoader.load(Pose.class, \"teleop.poses.blue.score\");\n");
    sb.append("    }\n");
    sb.append(
        "    return new MatchProfile(alliance, goalX, goalY, startPose, drinkPose, parkPose,"
            + " scorePose);\n");
    sb.append("  }\n");

    sb.append("}\n");
    return sb.toString();
  }

  private static class LeafPath {
    List<String> path;
    Object value;

    LeafPath(List<String> path, Object value) {
      this.path = path;
      this.value = value;
    }
  }

  @SuppressWarnings("unchecked")
  private static void collectLeafPaths(
      Map<String, Object> block, List<String> prefix, List<LeafPath> out) {
    for (Map.Entry<String, Object> entry : block.entrySet()) {
      List<String> curr = new ArrayList<>(prefix);
      curr.add(entry.getKey());
      if (entry.getValue() instanceof Map) {
        collectLeafPaths((Map<String, Object>) entry.getValue(), curr, out);
      } else {
        out.add(new LeafPath(curr, entry.getValue()));
      }
    }
  }

  @SuppressWarnings("unchecked")
  private static void generateNestedClass(
      StringBuilder sb,
      String className,
      Map<String, Object> block,
      Map<String, DocInfo> docs,
      String indent) {
    sb.append(indent).append("public static final class ").append(className).append(" {\n");
    for (Map.Entry<String, Object> entry : block.entrySet()) {
      String key = entry.getKey();
      Object val = entry.getValue();
      DocInfo doc = docs.get(key);

      if (doc != null && doc.desc != null && !doc.desc.isEmpty()) {
        sb.append(indent).append("  /**\n");
        sb.append(indent).append("   * ").append(doc.desc).append("\n");
        if (doc.min != null) sb.append(indent).append("   * Minimum: ").append(doc.min).append("\n");
        if (doc.max != null) sb.append(indent).append("   * Maximum: ").append(doc.max).append("\n");
        sb.append(indent).append("   */\n");
      }

      if (val instanceof Map) {
        Map<String, Object> subMap = (Map<String, Object>) val;
        if (subMap.keySet().containsAll(Arrays.asList("p", "i", "d"))
            && subMap.size() <= 4) {
          sb.append(indent)
              .append("  public com.qualcomm.robotcore.hardware.PIDFCoefficients ")
              .append(key)
              .append(";\n");
        } else {
          String subClassName = toClassName(key);
          generateNestedClass(sb, subClassName, subMap, docs, indent + "  ");
          sb.append(indent).append("  public ").append(subClassName).append(" ").append(key).append(";\n");
        }
      } else {
        String javaType = getJavaType(val, key, null, null);
        sb.append(indent).append("  public ").append(javaType).append(" ").append(key).append(";\n");
      }
    }
    sb.append(indent).append("}\n");
  }

  @SuppressWarnings("unchecked")
  private static void emitAllianceClass(
      StringBuilder sb,
      String className,
      Map<String, Object> poseSection,
      Map<String, Object> globalFields,
      Map<String, DocInfo> docs) {
    Set<String> poseKeys = new TreeSet<>();
    for (String alliance : Arrays.asList("blue", "red")) {
      if (poseSection.containsKey(alliance) && poseSection.get(alliance) instanceof Map) {
        poseKeys.addAll(((Map<String, Object>) Objects.requireNonNull(poseSection.get(alliance))).keySet());
      }
    }

    sb.append("  public static final class ").append(className).append(" {\n");
    for (String key : poseKeys) {
      DocInfo doc = docs.get(key);
      if (doc != null && doc.desc != null && !doc.desc.isEmpty()) {
        sb.append("    /**\n");
        sb.append("     * ").append(doc.desc).append("\n");
        sb.append("     */\n");
      }
      sb.append("    public Pose ").append(toCamelCase(key)).append(";\n");
    }

    if (globalFields != null && !globalFields.isEmpty()) {
      sb.append("\n    // Shared operational parameters (same for both alliances)\n");
      for (Map.Entry<String, Object> entry : globalFields.entrySet()) {
        String key = entry.getKey();
        DocInfo doc = docs.get(key);
        if (doc != null && doc.desc != null && !doc.desc.isEmpty()) {
          sb.append("    /**\n");
          sb.append("     * ").append(doc.desc).append("\n");
          sb.append("     */\n");
        }
        String javaType = getJavaType(entry.getValue(), key, null, null);
        sb.append("    public ").append(javaType).append(" ").append(toCamelCase(key)).append(";\n");
      }
    }
    sb.append("  }\n\n");
  }

  private static String getJavaType(
      Object val, String key, List<String> path, Map<String, Object> configData) {
    if (val instanceof String && isMirrorString((String) val)) {
      if (path != null && configData != null) {
        Object mirrorVal = resolveMirrorVal(path, configData);
        if (mirrorVal != null && !(mirrorVal instanceof String && isMirrorString((String) mirrorVal))) {
          return getJavaType(mirrorVal, key, path, configData);
        }
      }
      return "Pose";
    }
    if (val instanceof Boolean) return "boolean";
    if (val instanceof Number) {
      if (val instanceof Integer
          && (key.endsWith("_ms") || key.endsWith("_count") || key.endsWith("_id"))) {
        return "int";
      }
      return "double";
    }
    if (val instanceof List) {
      List<?> list = (List<?>) val;
      if (list.size() == 3) return "Pose";
      return "double[]";
    }
    return "String";
  }

  private static boolean isMirrorString(String val) {
    String v = val.trim().toLowerCase();
    return "m".equals(v) || "mirror".equals(v);
  }

  private static Object resolveMirrorVal(List<String> path, Map<String, Object> configData) {
    if (path == null || path.isEmpty()) return null;
    String leaf = path.get(path.size() - 1);

    String[][] pairs = {
      {"red", "blue"}, {"blue", "red"}, {"RED", "BLUE"}, {"BLUE", "RED"}
    };
    for (String[] pair : pairs) {
      List<String> mirrorSegments = new ArrayList<>();
      for (String s : path) {
        mirrorSegments.add(s.equals(pair[0]) ? pair[1] : s);
      }
      if (!mirrorSegments.equals(path)) {
        Object val = getPathValue(configData, mirrorSegments);
        if (val != null) return val;
      }
    }

    String leafLower = leaf.toLowerCase();
    for (String[] pair : new String[][] {{"red", "blue"}, {"blue", "red"}}) {
      String a = pair[0];
      String m = pair[1];
      String[][] variants = {
        {a + "_", m + "_"},
        {"_" + a, "_" + m},
        {a.toUpperCase() + "_", m.toUpperCase() + "_"},
        {"_" + a.toUpperCase(), "_" + m.toUpperCase()}
      };
      for (String[] var : variants) {
        if (leafLower.contains(var[0])) {
          String expectedLeaf = leafLower.replace(var[0], var[1]);
          List<String> mirrorSegments = new ArrayList<>(path.subList(0, path.size() - 1));
          mirrorSegments.add(expectedLeaf);
          Object val = getPathValue(configData, mirrorSegments);
          if (val != null) return val;
        }
      }
    }
    return null;
  }

  @SuppressWarnings("unchecked")
  private static Object getPathValue(Map<String, Object> data, List<String> path) {
    Object curr = data;
    for (String p : path) {
      if (curr instanceof Map) {
        curr = ((Map<String, Object>) curr).get(p);
      } else {
        return null;
      }
    }
    return curr;
  }

  private static String toClassName(String snakeStr) {
    String[] parts = snakeStr.split("_");
    StringBuilder sb = new StringBuilder();
    for (String p : parts) {
      if (!p.isEmpty()) {
        sb.append(Character.toUpperCase(p.charAt(0))).append(p.substring(1));
      }
    }
    return sb.toString();
  }

  private static String toCamelCase(String snakeStr) {
    String[] parts = snakeStr.split("_");
    StringBuilder sb = new StringBuilder(parts[0]);
    for (int i = 1; i < parts.length; i++) {
      if (!parts[i].isEmpty()) {
        sb.append(Character.toUpperCase(parts[i].charAt(0))).append(parts[i].substring(1));
      }
    }
    return sb.toString();
  }

  // --- JSON Schema Generation ---

  private static String generateSchemaJson(
      Map<String, Object> configData, Map<String, DocInfo> docs) {
    StringBuilder sb = new StringBuilder();
    sb.append("{\n");
    sb.append("  \"$schema\": \"http://json-schema.org/draft-07/schema#\",\n");
    sb.append("  \"title\": \"Robot Configuration Unified Schema\",\n");
    sb.append("  \"type\": \"object\",\n");
    sb.append("  \"properties\": {\n");

    List<Map.Entry<String, Object>> entries = new ArrayList<>(configData.entrySet());
    for (int i = 0; i < entries.size(); i++) {
      Map.Entry<String, Object> entry = entries.get(i);
      sb.append("    \"").append(entry.getKey()).append("\": ");
      appendSchemaProperty(sb, entry.getKey(), entry.getValue(), docs, "    ");
      if (i < entries.size() - 1) sb.append(",");
      sb.append("\n");
    }

    sb.append("  },\n");
    sb.append("  \"required\": [\n");
    for (int i = 0; i < entries.size(); i++) {
      sb.append("    \"").append(entries.get(i).getKey()).append("\"");
      if (i < entries.size() - 1) sb.append(",");
      sb.append("\n");
    }
    sb.append("  ]\n");
    sb.append("}\n");
    return sb.toString();
  }

  @SuppressWarnings("unchecked")
  private static void appendSchemaProperty(
      StringBuilder sb, String key, Object val, Map<String, DocInfo> docs, String indent) {
    DocInfo doc = docs.get(key);
    sb.append("{\n");

    boolean hasDesc = doc != null && doc.desc != null && !doc.desc.isEmpty();
    if (hasDesc) {
      sb.append(indent).append("  \"description\": \"").append(escapeJson(doc.desc)).append("\",\n");
    }

    if (val instanceof Boolean) {
      sb.append(indent).append("  \"type\": \"boolean\"\n");
    } else if (val instanceof Number) {
      sb.append(indent).append("  \"type\": \"number\"");
      if (doc != null && doc.min != null) {
        sb.append(",\n").append(indent).append("  \"minimum\": ").append(doc.min);
      }
      if (doc != null && doc.max != null) {
        sb.append(",\n").append(indent).append("  \"maximum\": ").append(doc.max);
      }
      sb.append("\n");
    } else if (val instanceof List || (val instanceof String && isMirrorString((String) val))) {
      if ((val instanceof List && ((List<?>) val).size() == 3)
          || (val instanceof String && isMirrorString((String) val))) {
        sb.append(indent).append("  \"oneOf\": [\n");
        sb.append(indent)
            .append(
                "    {\"type\": \"array\", \"items\": {\"type\": \"number\"}, \"minItems\": 3,"
                    + " \"maxItems\": 3},\n");
        sb.append(indent)
            .append("    {\"type\": \"string\", \"enum\": [\"m\", \"M\", \"mirror\", \"MIRROR\"]}\n");
        sb.append(indent).append("  ]\n");
      } else {
        sb.append(indent).append("  \"type\": \"array\",\n");
        sb.append(indent).append("  \"items\": {\"type\": \"number\"}\n");
      }
    } else if (val instanceof Map) {
      Map<String, Object> subMap = (Map<String, Object>) val;
      sb.append(indent).append("  \"type\": \"object\",\n");
      sb.append(indent).append("  \"properties\": {\n");

      List<Map.Entry<String, Object>> subEntries = new ArrayList<>(subMap.entrySet());
      for (int i = 0; i < subEntries.size(); i++) {
        Map.Entry<String, Object> subEntry = subEntries.get(i);
        sb.append(indent).append("    \"").append(subEntry.getKey()).append("\": ");
        appendSchemaProperty(sb, subEntry.getKey(), subEntry.getValue(), docs, indent + "    ");
        if (i < subEntries.size() - 1) sb.append(",");
        sb.append("\n");
      }

      sb.append(indent).append("  },\n");
      sb.append(indent).append("  \"required\": [\n");
      for (int i = 0; i < subEntries.size(); i++) {
        sb.append(indent).append("    \"").append(subEntries.get(i).getKey()).append("\"");
        if (i < subEntries.size() - 1) sb.append(",");
        sb.append("\n");
      }
      sb.append(indent).append("  ]\n");
    } else {
      sb.append(indent).append("  \"type\": \"string\"\n");
    }

    sb.append(indent).append("}");
  }

  private static String escapeJson(String str) {
    return str.replace("\\", "\\\\").replace("\"", "\\\"");
  }

  private static String readFileToString(File file) throws IOException {
    try (FileInputStream fis = new FileInputStream(file);
        InputStreamReader isr = new InputStreamReader(fis, StandardCharsets.UTF_8);
        BufferedReader reader = new BufferedReader(isr)) {
      StringBuilder sb = new StringBuilder((int) file.length());
      char[] buffer = new char[8192];
      int len;
      while ((len = reader.read(buffer)) != -1) {
        sb.append(buffer, 0, len);
      }
      return sb.toString();
    }
  }



  private static boolean writeIfChanged(File file, String newContent) {
    if (file.exists()) {
      try {
        String existingContent = readFileToString(file);
        if (existingContent.equals(newContent)) {
          return false;
        }
      } catch (IOException ignored) {
      }
    }
    File parent = file.getParentFile();
    if (parent != null && !parent.exists()) {
      boolean created = parent.mkdirs();
      if (!created && !parent.exists()) {
        throw new RuntimeException("Failed to create directory: " + parent.getAbsolutePath());
      }
    }

    try (FileOutputStream fos = new FileOutputStream(file);

        OutputStreamWriter writer = new OutputStreamWriter(fos, StandardCharsets.UTF_8)) {
      writer.write(newContent);
    } catch (IOException e) {
      throw new RuntimeException("Failed to write file: " + file.getAbsolutePath(), e);
    }
    return true;
  }
}


