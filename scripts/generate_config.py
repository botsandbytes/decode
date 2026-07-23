#!/usr/bin/env python3
import os
import json
import re

# Paths
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(BASE_DIR, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml")
DOCS_PATH = os.path.join(BASE_DIR, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config-docs.yaml")
JAVA_OUTPUT_PATH = os.path.join(BASE_DIR, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.java")
SCHEMA_OUTPUT_PATH = os.path.join(BASE_DIR, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config-schema.json")


def parse_yaml(filepath):
    """
    Parses a simple YAML file into a nested dictionary.
    Handles indentation, simple types, and inline lists [a, b, c].
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    root = {}
    stack = [(-1, root)]
    
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
            
        # Determine indentation
        indent = len(line) - len(line.lstrip())
        
        # Parse key/value
        if ':' in stripped:
            parts = stripped.split(':', 1)
            key = parts[0].strip()
            val_str = parts[1].strip()
            
            # Pop stack back to parent level
            while stack and stack[-1][0] >= indent:
                stack.pop()
                
            parent_map = stack[-1][1]
            
            if val_str:
                # Leaf key with value
                value = parse_value(val_str)
                parent_map[key] = value
            else:
                # Nesting block
                new_map = {}
                parent_map[key] = new_map
                stack.append((indent, new_map))
                
    return root

def parse_value(val_str):
    """Parses primitive types and lists in yaml values."""
    if val_str.startswith('[') and val_str.endswith(']'):
        # List: [1, 2, 3]
        elements = val_str[1:-1].split(',')
        return [parse_value(e.strip()) for e in elements]
    
    if val_str.lower() == 'true':
        return True
    if val_str.lower() == 'false':
        return False
    
    # Try parsing number
    try:
        if '.' in val_str:
            return float(val_str)
        else:
            return int(val_str)
    except ValueError:
        pass
    
    # Strip quotes if string
    if (val_str.startswith('"') and val_str.endswith('"')) or (val_str.startswith("'") and val_str.endswith("'")):
        return val_str[1:-1]
        
    return val_str

def load_docs(filepath):
    """Loads config-docs.yaml descriptions."""
    if not os.path.exists(filepath):
        return {}
    
    raw_docs = parse_yaml(filepath)
    docs = {}
    
    for key, val in raw_docs.items():
        if isinstance(val, dict):
            docs[key] = {
                "desc": val.get("desc", ""),
                "min": val.get("min"),
                "max": val.get("max")
            }
        else:
            docs[key] = {
                "desc": str(val),
                "min": None,
                "max": None
            }
    return docs

def to_camel_case(snake_str):
    """Converts snake_case to camelCase."""
    components = snake_str.split('_')
    return components[0] + ''.join(x.title() for x in components[1:])

def resolve_mirror_val(path, config_data):
    if not path or not config_data:
        return None
    segments = path[:]
    leaf = segments[-1]
    
    # 1. Path segment replacement (e.g. auto_poses.normal.red.start -> auto_poses.normal.blue.start)
    for alliance, mirror in (("red", "blue"), ("blue", "red"), ("RED", "BLUE"), ("BLUE", "RED")):
        mirror_segments = [mirror if s == alliance else s for s in segments]
        if mirror_segments != segments:
            curr = config_data
            for p in mirror_segments:
                if isinstance(curr, dict) and p in curr:
                    curr = curr[p]
                else:
                    curr = None
                    break
            if curr is not None:
                return curr

    # 2. Leaf key replacement (e.g. red_base_power -> blue_base_power)
    leaf_lower = leaf.lower()
    for alliance, mirror in (("red", "blue"), ("blue", "red")):
        variants = [
            (alliance + "_", mirror + "_"),
            ("_" + alliance, "_" + mirror),
            (alliance.upper() + "_", mirror.upper() + "_"),
            ("_" + alliance.upper(), "_" + mirror.upper()),
        ]
        for a_pat, m_pat in variants:
            if a_pat in leaf_lower:
                expected_leaf = leaf_lower.replace(a_pat, m_pat)
                mirror_segments = segments[:-1] + [expected_leaf]
                curr = config_data
                for p in mirror_segments:
                    if isinstance(curr, dict) and p in curr:
                        curr = curr[p]
                    else:
                        curr = None
                        break
                if curr is not None:
                    return curr
    return None

def get_java_type(val, key="", path=None, config_data=None):
    """Resolves Java type for a value."""
    if isinstance(val, str) and val.lower() in ['m', 'mirror']:
        if path and config_data:
            mirror_val = resolve_mirror_val(path, config_data)
            if mirror_val is not None and not (isinstance(mirror_val, str) and mirror_val.lower() in ['m', 'mirror']):
                return get_java_type(mirror_val, key, path, config_data)
        return "Pose"
    if isinstance(val, bool):
        return "boolean"
    if isinstance(val, (int, float)):
        if isinstance(val, int) and (key.endswith("_ms") or key.endswith("_count") or key.endswith("_id")):
            return "int"
        return "double"
    if isinstance(val, list):
        if len(val) == 3:
            return "Pose"
        return "double[]"
    return "String"

def get_java_literal(val):
    """Formats Java literal value."""
    if isinstance(val, bool):
        return "true" if val else "false"
    if isinstance(val, float):
        return f"{val}"
    if isinstance(val, int):
        return f"{val}"
    if isinstance(val, list):
        if len(val) == 3:
            return f"new Pose({val[0]}, {val[1]}, Math.toRadians({val[2]}))"
        return "{" + ", ".join(str(v) for v in val) + "}"
    if isinstance(val, str) and val.lower() in ['m', 'mirror']:
        return "null"
    return f'"{val}"'

def collect_leaf_paths(block, prefix_list=None):
    if prefix_list is None:
        prefix_list = []
    
    paths = []
    for key, val in block.items():
        current_path = prefix_list + [key]
        if isinstance(val, dict):
            paths.extend(collect_leaf_paths(val, current_path))
        else:
            paths.append((current_path, val))
    return paths

def generate_java_class(config_data, docs):
    """Generates the config.java class with dynamic reload() support."""
    lines = [
        "// AUTO-GENERATED FILE - DO NOT EDIT DIRECTLY",
        "package org.firstinspires.ftc.teamcode.robot.config;",
        "",
        "import com.pedropathing.geometry.Pose;",
        "import org.firstinspires.ftc.teamcode.config.ConfigLoader;",
        "import org.firstinspires.ftc.teamcode.records.Alliance;",
        "import org.firstinspires.ftc.teamcode.records.Field;",
        "import org.firstinspires.ftc.teamcode.records.MatchProfile;",
        "",
        "public final class config {",
        "  private config() {}",
        ""
    ]
    
    # Collect sections and leaf paths
    sections = [s for s, v in config_data.items() if isinstance(v, dict) and s not in ["auto_poses"]]
    leaf_paths = []
    for s in sections:
        leaf_paths.extend(collect_leaf_paths(config_data[s], [s]))

    # Generate static fields for section objects
    for section_key in sections:
        class_name = section_key.title().replace("_", "")
        lines.append(f"  public static {class_name} {section_key};")
    
    lines.append("")

    # Generate static fields for flat uppercase aliases
    for path, val in leaf_paths:
        alias_name = "_".join(p.upper() for p in path)
        java_type = get_java_type(val, path[-1], path, config_data)
        lines.append(f"  public static {java_type} {alias_name};")
        
    lines.append("")
    lines.append("  static {")
    lines.append("    reload();")
    lines.append("  }")
    lines.append("")
    lines.append("  public static synchronized void reload() {")
    lines.append("    ConfigLoader.reload();")
    for section_key in sections:
        class_name = section_key.title().replace("_", "")
        lines.append(f"    {section_key} = ConfigLoader.load({class_name}.class, \"{section_key}\");")
    
    for path, val in leaf_paths:
        java_path = ".".join(path)
        alias_name = "_".join(p.upper() for p in path)
        lines.append(f"    {alias_name} = {java_path};")
    lines.append("  }")
    lines.append("")
    
    # Generate nested static classes
    for section_key, section_val in config_data.items():
        if isinstance(section_val, dict) and section_key not in ["auto_poses"]:
            class_name = section_key.title().replace("_", "")
            lines.extend(generate_nested_class(class_name, section_val, docs, "  "))
            lines.append("")
            
    # Generate flat AutoPoses classes (blue/red sibling detection → merge global auto fields)
    if "auto_poses" in config_data:
        auto_poses = config_data["auto_poses"]
        auto_global = config_data.get("auto", {})

        # Separate normal auto fields (no opposite_ prefix) from opposite auto fields
        normal_global = {k: v for k, v in auto_global.items() if not k.startswith("opposite_")}
        opposite_global = {
            k[len("opposite_"):]: v
            for k, v in auto_global.items()
            if k.startswith("opposite_")
        }

        def emit_alliance_class(class_name, pose_section, global_fields, pose_path_template):
            """Emits a merged class with alliance-specific poses + global operational fields."""
            pose_keys = set()
            for alliance in ["blue", "red"]:
                if alliance in pose_section:
                    pose_keys.update(pose_section[alliance].keys())

            lines.append(f"  public static final class {class_name} {{")

            # Pose fields (alliance-specific)
            for key in sorted(pose_keys):
                doc = docs.get(key, {})
                desc = doc.get("desc", "")
                if desc:
                    lines.append("    /**")
                    lines.append(f"     * {desc}")
                    lines.append("     */")
                camel_key = to_camel_case(key)
                lines.append(f"    public Pose {camel_key};")

            # Global operational fields merged in
            if global_fields:
                lines.append("")
                lines.append("    // Shared operational parameters (same for both alliances)")
                for key, val in global_fields.items():
                    doc = docs.get(key, {})
                    desc = doc.get("desc", "")
                    if desc:
                        lines.append("    /**")
                        lines.append(f"     * {desc}")
                        lines.append("     */")
                    java_type = get_java_type(val, key)
                    camel_key = to_camel_case(key)
                    lines.append(f"    public {java_type} {camel_key};")

            lines.append("  }")
            lines.append("")

        # 1. Normal Auto
        if "normal" in auto_poses:
            emit_alliance_class("NormalAuto", auto_poses["normal"], normal_global,
                                "auto_poses.normal.{alliance}")

        # 2. Opposite Auto
        if "opposite" in auto_poses:
            emit_alliance_class("OppositeAuto", auto_poses["opposite"], opposite_global,
                                "auto_poses.opposite.{alliance}")
            
    # Add LoadMatchProfile helper (matching the original loadMatchProfile)
    lines.extend([
        "  public static MatchProfile loadMatchProfile(Alliance alliance) {",
        "    String allianceStr = alliance == Alliance.RED ? \"red\" : \"blue\";",
        "    double goalX = Field.getGoalX(alliance);",
        "    double goalY = Field.getGoalY(alliance);",
        "    Pose startPose = ConfigLoader.load(Pose.class, \"teleop.poses.\" + allianceStr + \".start\");",
        "    Pose drinkPose = ConfigLoader.load(Pose.class, \"teleop.poses.\" + allianceStr + \".drink\");",
        "    Pose parkPose = ConfigLoader.load(Pose.class, \"teleop.poses.\" + allianceStr + \".park\");",
        "    Pose scorePose;",
        "    if (alliance == Alliance.RED) {",
        "      Pose redScore = ConfigLoader.load(Pose.class, \"teleop.poses.red.score\");",
        "      scorePose = org.firstinspires.ftc.teamcode.robot.Turret.alignPose(redScore.getX(), redScore.getY(), goalX + 2.5, goalY);",
        "    } else {",
        "      scorePose = ConfigLoader.load(Pose.class, \"teleop.poses.blue.score\");",
        "    }",
        "    return new MatchProfile(alliance, goalX, goalY, startPose, drinkPose, parkPose, scorePose);",
        "  }"
    ])
    
    lines.append("}")
    
    return "\n".join(lines) + "\n"

def generate_nested_class(class_name, block, docs, indent):
    """Recursively generates nested configuration classes."""
    lines = [f"{indent}public static final class {class_name} {{"]
    
    for key, val in block.items():
        doc = docs.get(key, {})
        desc = doc.get("desc", "")
        
        # Generate Javadocs
        if desc:
            lines.append(f"{indent}  /**")
            lines.append(f"{indent}   * {desc}")
            if doc.get("min") is not None:
                lines.append(f"{indent}   * Minimum: {doc['min']}")
            if doc.get("max") is not None:
                lines.append(f"{indent}   * Maximum: {doc['max']}")
            lines.append(f"{indent}   */")
            
        if isinstance(val, dict):
            if set(val.keys()).issubset({'p', 'i', 'd', 'f'}):
                lines.append(f"{indent}  public com.qualcomm.robotcore.hardware.PIDFCoefficients {key};")
            else:
                # Nested class block
                sub_class_name = key.title().replace("_", "")
                lines.extend(generate_nested_class(sub_class_name, val, docs, indent + "  "))
                lines.append(f"{indent}  public {sub_class_name} {key};")
        else:
            # Field definition
            java_type = get_java_type(val, key)
            java_lit = get_java_literal(val)
            lines.append(f"{indent}  public {java_type} {key};")
            
    lines.append(f"{indent}}}")
    return lines

def generate_json_schema(config_data, docs):
    """Generates the config-schema.json schema."""
    schema = {
        "$schema": "http://json-schema.org/draft-07/schema#",
        "title": "Robot Configuration Unified Schema",
        "type": "object",
        "properties": {},
        "required": []
    }
    
    for key, val in config_data.items():
        schema["properties"][key] = build_schema_property(key, val, docs)
        schema["required"].append(key)
        
    return json.dumps(schema, indent=2) + "\n"

def build_schema_property(key, val, docs):
    """Recursively builds schema validation nodes."""
    doc = docs.get(key, {})
    desc = doc.get("desc", "")
    
    prop = {}
    if desc:
        prop["description"] = desc
        
    if isinstance(val, bool):
        prop["type"] = "boolean"
    elif isinstance(val, (int, float)):
        prop["type"] = "number"
        if doc.get("min") is not None:
            prop["minimum"] = doc["min"]
        if doc.get("max") is not None:
            prop["maximum"] = doc["max"]
    elif isinstance(val, list) or (isinstance(val, str) and val.lower() in ['m', 'mirror']):
        if (isinstance(val, list) and len(val) == 3) or (isinstance(val, str) and val.lower() in ['m', 'mirror']):
            prop["oneOf"] = [
                {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
                {"type": "string", "enum": ["m", "M", "mirror", "MIRROR"]}
            ]
        else:
            prop["type"] = "array"
            prop["items"] = {"type": "number"}
    elif isinstance(val, dict):
        prop["type"] = "object"
        prop["properties"] = {}
        prop["required"] = []
        for sub_key, sub_val in val.items():
            prop["properties"][sub_key] = build_schema_property(sub_key, sub_val, docs)
            prop["required"].append(sub_key)
    else:
        prop["type"] = "string"
        
    return prop

def main():
    if not os.path.exists(CONFIG_PATH):
        print(f"Error: config.yaml not found at {CONFIG_PATH}")
        return
        
    config_data = parse_yaml(CONFIG_PATH)
    docs = load_docs(DOCS_PATH)
    
    # 1. Generate config.java
    java_content = generate_java_class(config_data, docs)
    with open(JAVA_OUTPUT_PATH, 'w') as f:
        f.write(java_content)
    print(f"Generated {JAVA_OUTPUT_PATH}")

    # 2. Generate config-schema.json
    schema_content = generate_json_schema(config_data, docs)
    with open(SCHEMA_OUTPUT_PATH, 'w') as f:
        f.write(schema_content)
    print(f"Generated {SCHEMA_OUTPUT_PATH}")

if __name__ == "__main__":
    main()
