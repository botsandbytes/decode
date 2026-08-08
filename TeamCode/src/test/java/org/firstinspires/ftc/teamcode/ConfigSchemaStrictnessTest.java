package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.networknt.schema.Schema;
import com.networknt.schema.SchemaRegistry;
import com.networknt.schema.SpecificationVersion;
import java.io.InputStream;
import java.util.Map;
import org.junit.Test;
import org.yaml.snakeyaml.Yaml;
import tools.jackson.databind.JsonNode;
import tools.jackson.databind.ObjectMapper;

/**
 * Proves the GENERATED config-schema.json actually enforces type / range / required constraints —
 * i.e. it is not the old hollow {@code anyOf} schema that passed almost anything. Each test mutates
 * a fresh copy of the real config and asserts the schema rejects it.
 */
public class ConfigSchemaStrictnessTest {

  private Schema schema() {
    InputStream schemaStream =
        getClass()
            .getResourceAsStream(
                "/org/firstinspires/ftc/teamcode/robot/config/generated/config-schema.json");
    return SchemaRegistry.withDefaultDialect(SpecificationVersion.DRAFT_7).getSchema(schemaStream);
  }

  private Map<String, Object> freshConfig() {
    InputStream yamlStream =
        getClass().getResourceAsStream("/org/firstinspires/ftc/teamcode/robot/config/config.yaml");
    return new Yaml().load(yamlStream);
  }

  private boolean valid(Map<String, Object> cfg) {
    JsonNode node = new ObjectMapper().valueToTree(cfg);
    return schema().validate(node).isEmpty();
  }

  @SuppressWarnings("unchecked")
  private Map<String, Object> child(Map<String, Object> m, String key) {
    return (Map<String, Object>) m.get(key);
  }

  @Test
  public void baseline_realConfigIsValid() {
    assertTrue("real config.yaml should validate against its schema", valid(freshConfig()));
  }

  @Test
  public void rejects_wrongType() {
    Map<String, Object> cfg = freshConfig();
    child(cfg, "shooter").put("max_rpm", "not_a_number"); // schema says number
    assertFalse("schema must reject a string where a number is required", valid(cfg));
  }

  @Test
  public void rejects_belowMinimum() {
    Map<String, Object> cfg = freshConfig();
    // shooter.max_rpm has "minimum": 0.0 in the generated schema
    child(cfg, "shooter").put("max_rpm", -5.0);
    assertFalse("schema must reject a value below its documented minimum", valid(cfg));
  }

  @Test
  public void rejects_aboveMaximum() {
    Map<String, Object> cfg = freshConfig();
    // turret.max_power_output has "maximum": 1.0
    child(cfg, "turret").put("max_power_output", 5.0);
    assertFalse("schema must reject a value above its documented maximum", valid(cfg));
  }

  @Test
  public void rejects_missingRequiredKey() {
    Map<String, Object> cfg = freshConfig();
    child(cfg, "shooter").remove("max_rpm");
    assertFalse("schema must reject config missing a required key", valid(cfg));
  }

  @Test
  public void rejects_missingRequiredSection() {
    Map<String, Object> cfg = freshConfig();
    cfg.remove("sentinel");
    assertFalse("schema must reject config missing a whole required section", valid(cfg));
  }

  @Test
  public void nested_typeViolationIsCaught() {
    Map<String, Object> cfg = freshConfig();
    child(child(cfg, "turret"), "pidf").put("p", "oops");
    assertFalse("schema must recurse into nested objects and catch type errors", valid(cfg));
  }

  @Test
  public void controlMutation_stillValid() {
    // Sanity: a legal in-range change to a real key must still validate, so the failures above
    // are about the constraint, not about mutation breaking the map.
    Map<String, Object> cfg = freshConfig();
    child(cfg, "shooter").put("max_rpm", 1600.0);
    assertTrue(valid(cfg));
  }
}
