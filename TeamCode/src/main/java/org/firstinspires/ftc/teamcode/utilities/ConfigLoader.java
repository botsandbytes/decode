package org.firstinspires.ftc.teamcode.utilities;

import org.yaml.snakeyaml.Yaml;
import java.io.InputStream;
import java.util.Map;
import java.util.HashMap;

public class ConfigLoader {
    private static final Map<String, Object> config = new HashMap<>();

    static {
        try {
            Yaml yaml = new Yaml();
            String[] files = {"subsystems.yaml", "safety.yaml", "tuning.yaml", "auto.yaml"};
            for (String file : files) {
                InputStream inputStream = ConfigLoader.class.getResourceAsStream("/org/firstinspires/ftc/teamcode/robot/config/" + file);
                if (inputStream == null) {
                    throw new RuntimeException("Could not find " + file + " in classpath resources");
                }
                Map<String, Object> loaded = yaml.load(inputStream);
                if (loaded != null) {
                    mergeMaps(config, loaded);
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load configuration YAMLs", e);
        }
    }

    @SuppressWarnings("unchecked")
    private static void mergeMaps(Map<String, Object> target, Map<String, Object> source) {
        for (Map.Entry<String, Object> entry : source.entrySet()) {
            String key = entry.getKey();
            Object value = entry.getValue();
            if (value instanceof Map && target.get(key) instanceof Map) {
                mergeMaps((Map<String, Object>) target.get(key), (Map<String, Object>) value);
            } else {
                target.put(key, value);
            }
        }
    }

    @SuppressWarnings("unchecked")
    public static <T> T get(String path) {
        String[] parts = path.split("\\.");
        Object current = config;
        for (String part : parts) {
            if (current instanceof Map) {
                current = ((Map<String, Object>) current).get(part);
            } else {
                return null;
            }
        }
        return (T) current;
    }

    public static double getDouble(String path) {
        Number val = get(path);
        return val != null ? val.doubleValue() : 0.0;
    }

    public static int getInt(String path) {
        Number val = get(path);
        return val != null ? val.intValue() : 0;
    }

    public static String getString(String path) {
        return get(path);
    }

    public static boolean getBoolean(String path) {
        Boolean val = get(path);
        return val != null && val;
    }
}
