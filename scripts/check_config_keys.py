#!/usr/bin/env python3
"""
Compile-time config key similarity checker.
Detects:
  1. Alliance asymmetry  – a key mentions 'red' or 'blue' but the mirror key is absent.
  2. Fuzzy duplicates    – two flat keys with Levenshtein distance <= 2 that look accidental.

Violations exit non-zero so the Gradle verifyBuild chain fails loudly.
Suppression: add  suppress_similarity_check: true  under the key in config-docs.yaml.
"""
import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(BASE_DIR, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config.yaml")
DOCS_PATH   = os.path.join(BASE_DIR, "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/config-docs.yaml")

# ──────────────────────────────────────────────
# Minimal YAML parser (reused from generate_config.py style)
# ──────────────────────────────────────────────
def parse_yaml(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()
    root = {}
    stack = [(-1, root)]
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue
        indent = len(line) - len(line.lstrip())
        if ':' in stripped:
            parts = stripped.split(':', 1)
            key = parts[0].strip()
            val_str = parts[1].strip()
            while stack and stack[-1][0] >= indent:
                stack.pop()
            parent_map = stack[-1][1]
            if val_str:
                parent_map[key] = val_str
            else:
                new_map = {}
                parent_map[key] = new_map
                stack.append((indent, new_map))
    return root


def flatten_keys(d, prefix=""):
    """Recursively collect all leaf keys as flat dot-separated paths."""
    keys = []
    for k, v in d.items():
        full = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            keys.extend(flatten_keys(v, full))
        else:
            keys.append(full)
    return keys


def leaf_names(flat_keys):
    """Return just the final segment of each path."""
    return [k.split(".")[-1] for k in flat_keys]


def levenshtein(a, b):
    a, b = a.lower(), b.lower()
    if a == b:
        return 0
    m, n = len(a), len(b)
    dp = list(range(n + 1))
    for i in range(1, m + 1):
        prev = dp[:]
        dp[0] = i
        for j in range(1, n + 1):
            if a[i-1] == b[j-1]:
                dp[j] = prev[j-1]
            else:
                dp[j] = 1 + min(prev[j], dp[j-1], prev[j-1])
    return dp[n]


def load_suppressions(docs_path):
    """Returns a set of key names that have suppress_similarity_check: true."""
    suppressed = set()
    if not os.path.exists(docs_path):
        return suppressed
    docs = parse_yaml(docs_path)
    for key, val in docs.items():
        if isinstance(val, dict) and val.get("suppress_similarity_check") in ("true", True):
            suppressed.add(key)
    return suppressed


# ──────────────────────────────────────────────
# Check 1: Alliance asymmetry
# ──────────────────────────────────────────────
def check_alliance_symmetry(flat_keys, suppressed):
    errors = []
    names = {k.split(".")[-1]: k for k in flat_keys}

    for leaf, full_path in names.items():
        leaf_lower = leaf.lower()
        if leaf in suppressed:
            continue

        # Check for 'red' or 'blue' as a word boundary (prefix, suffix, or infix)
        for alliance, mirror in [("red", "blue"), ("blue", "red")]:
            variants = [
                (alliance + "_", mirror + "_"),           # red_drink  → blue_drink
                ("_" + alliance, "_" + mirror),           # drink_red  → drink_blue
                (alliance.upper() + "_", mirror.upper() + "_"),
                ("_" + alliance.upper(), "_" + mirror.upper()),
            ]
            for a_pat, m_pat in variants:
                if a_pat in leaf_lower:
                    expected_leaf = leaf_lower.replace(a_pat, m_pat)
                    # Check if any key has the mirror name
                    mirror_exists = any(
                        k.split(".")[-1].lower() == expected_leaf for k in flat_keys
                    )
                    if not mirror_exists and expected_leaf not in suppressed:
                        errors.append(
                            f"  Alliance asymmetry: '{leaf}' found (at {full_path})\n"
                            f"    but no mirror '{expected_leaf}' exists.\n"
                            f"    → Add the mirror key, or add suppress_similarity_check: true\n"
                            f"      under '{leaf}' in config-docs.yaml"
                        )
    return errors


# ──────────────────────────────────────────────
# Check 1b: Alliance asymmetry in PATH segments
# ──────────────────────────────────────────────
def check_path_alliance_symmetry(flat_keys, suppressed):
    """
    Catches asymmetry where 'red'/'blue' is a whole path SEGMENT rather than part
    of the leaf name — e.g. auto_poses.normal.red.pickup2_end vs
    auto_poses.normal.blue.pickup2_end. This is where most alliance-specific config
    actually lives (the pose trees), which the leaf-name check above does not cover.

    Suppress a legitimately single-alliance pose by adding
    suppress_similarity_check: true under its leaf name in config-docs.yaml.
    """
    errors = []
    key_set = set(flat_keys)
    for key in flat_keys:
        segments = key.split(".")
        leaf = segments[-1]
        if leaf in suppressed:
            continue
        for alliance, mirror in (("red", "blue"), ("blue", "red")):
            mirror_segments = [mirror if s == alliance else s for s in segments]
            if mirror_segments == segments:
                continue  # no bare 'red'/'blue' segment in this path
            mirror_key = ".".join(mirror_segments)
            if mirror_key not in key_set:
                errors.append(
                    f"  Alliance path asymmetry: '{key}'\n"
                    f"    but no mirror '{mirror_key}' exists.\n"
                    f"    → Add the mirror key, or add suppress_similarity_check: true\n"
                    f"      under '{leaf}' in config-docs.yaml"
                )
    return errors


# ──────────────────────────────────────────────
# Check 2: Fuzzy near-duplicates
# ──────────────────────────────────────────────
def check_fuzzy_duplicates(flat_keys, suppressed):
    """
    Flags keys that differ by 1-2 characters in a way that looks accidental.
    Structural pairs (min/max, x/y/z, red/blue, near/far, etc.) are normalized
    away before comparison so they never trigger — no suppression needed for those.
    """
    import re

    # Token sets that represent intentional structural pairings.
    # If two keys differ only by swapping tokens within one of these sets,
    # they are structural siblings, not typos.
    STRUCTURAL_PAIRS = [
        {'min', 'max'},
        {'x', 'y', 'z'},
        {'red', 'blue'},
        {'near', 'far'},
        {'slow', 'hard'},
        {'start', 'end', 'cp'},
        {'up', 'down'},
        {'left', 'right'},
        {'front', 'back'},
        {'roll', 'pitch', 'yaw'},
        {'goal', 'size'},
    ]

    def normalize(name):
        """Replace each word in the key with a canonical placeholder if it belongs
        to a structural pair set, so structural siblings collapse to the same string."""
        parts = re.split(r'[_\s]', name.lower())
        out = []
        for part in parts:
            replaced = False
            for i, pair_set in enumerate(STRUCTURAL_PAIRS):
                if part in pair_set:
                    out.append(f'__PAIR{i}__')
                    replaced = True
                    break
            if not replaced:
                out.append(part)
        return '_'.join(out)

    warnings = []
    names = [k.split('.')[-1] for k in flat_keys]
    seen_pairs = set()

    for i, a in enumerate(names):
        for j, b in enumerate(names):
            if i >= j:
                continue
            pair = tuple(sorted([a, b]))
            if pair in seen_pairs:
                continue
            seen_pairs.add(pair)
            if a in suppressed or b in suppressed:
                continue
            # Skip very short keys — single-letter PIDF constants are intentionally dense
            if len(a) <= 2 or len(b) <= 2:
                continue
            if a.lower() == b.lower():
                continue
            # Skip pairs that differ only in embedded digits (pickup1_cp vs pickup2_cp)
            if re.sub(r'\d+', '', a.lower()) == re.sub(r'\d+', '', b.lower()):
                continue
            # Skip structural siblings — same key shape, different axis/bound token
            if normalize(a) == normalize(b):
                continue
            dist = levenshtein(a, b)
            if dist <= 2 and dist > 0:
                warnings.append(
                    f"  Fuzzy duplicate: '{a}' and '{b}' differ by only {dist} character(s).\n"
                    f"    → Confirm these are intentionally distinct, or suppress with\n"
                    f"      suppress_similarity_check: true in config-docs.yaml"
                )
    return warnings


# ──────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────
def main():
    config = parse_yaml(CONFIG_PATH)
    flat_keys = flatten_keys(config)
    suppressed = load_suppressions(DOCS_PATH)

    errors   = check_alliance_symmetry(flat_keys, suppressed)
    errors  += check_path_alliance_symmetry(flat_keys, suppressed)
    warnings = check_fuzzy_duplicates(flat_keys, suppressed)

    all_ok = True

    if errors:
        all_ok = False
        print("CONFIG CHECK ERRORS:")
        for e in errors:
            print(e)
        print()

    if warnings:
        print("CONFIG CHECK WARNINGS (not build-blocking):")
        for w in warnings:
            print(w)
        print()

    if all_ok and not warnings:
        print("Config key check passed.")

    # Only hard-fail on errors, not warnings
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
