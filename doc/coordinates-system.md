# Coordinate Systems

This document describes the two coordinate systems relevant to this codebase:

1. The **Pedro Pathing coordinate system** — used by the Pedro Pathing library.
   **This is the system the localization/odometry feature now uses:**
   `DriveEncoderIMULocalizer` implements PP's `Localizer` and reports poses as
   PP `Pose` objects (default `PedroCoordinates`), and `TeleOpControlLinearOpMode`
   consumes them directly. PP is also planned for autonomous path following.
2. The **FTC SDK / FIRST Tech Challenge field coordinate system** — the official
   standard defined by *FIRST*, documented here for reference (it is what the SDK
   `Pose2D`, the IMU, AprilTag localization, and most sensors report in).

They are both right-handed systems, but they differ in **where the origin sits**
and **how the axes are oriented relative to the physical field**, so a pose is
*not* portable between them without conversion.

> **History:** an earlier version of the localizer tracked pose in the FTC field
> frame with hand-rolled integration. It was replaced by the Pedro Pathing
> `Localizer` implementation so we can swap in a Pinpoint/dead-wheel localizer and
> reuse PP for autonomous without a coordinate-frame change.

---

## 1. FTC SDK Field Coordinate System (reference)

Source: [FIRST Tech Challenge — Field Coordinate System Definition](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html)
(mirrored as the [official field coordinate system PDF](https://acmerobotics.github.io/ftc-dashboard/official_field_coord_sys.pdf)).

### Origin
`(0, 0, 0)` is at the **center of the field**, where the four center floor tiles
meet, resting on the **top surface of the floor mat**. Because the field is a
12 ft × 12 ft (≈ 144 in × 144 in) square, each axis runs from about **−72 in to
+72 in**.

### Axes (viewed standing at the Red Wall, looking at the origin)
| Axis | Direction | Positive sense |
|------|-----------|----------------|
| **X** | Runs left–right, **parallel** to the Red Wall | Increases **to the right** |
| **Y** | Runs out–in, **perpendicular** to the Red Wall | Increases **away** from the Red Wall |
| **Z** | Runs vertically | Increases **upward** |

### Heading / rotation
- Follows the **right-hand rule** about the **+Z axis** (looking down at the
  field from above).
- **Positive rotation is counter-clockwise (CCW)**; a robot spinning clockwise
  is making a *negative* rotation about Z.
- Heading is what an IMU reports as **yaw**.
- Convention used in this codebase: at **heading `0`** the robot's forward
  direction points along **field +X**; at **heading `+90°` (π/2 rad)** the
  robot's forward points along **field +Y**. Robot-forward rotates from +X
  toward +Y as heading increases (CCW), consistent with the right-hand rule.

### Units
Any consistent length unit (inches or millimeters). The SDK's
`org.firstinspires.ftc.robotcore.external.navigation.Pose2D` is **unit-tagged**:
you construct it with a `DistanceUnit` + `AngleUnit` and read it back in whatever
unit you ask for.

### Robot-frame convention (for odometry integration)
The field doc does not define a robot-relative frame, but to integrate wheel
motion we use the natural right-handed robot frame consistent with the above:
- robot **+x** = forward
- robot **+y** = left
- robot **+z** = up (shared with the field)

At heading `h`, robot-frame motion maps into the field frame by:

```
dX_field = dForward * cos(h) − dLeft * sin(h)
dY_field = dForward * sin(h) + dLeft * cos(h)
h        += dHeading            // CCW positive
```

---

## 2. Pedro Pathing Coordinate System (the one we use)

Source: [Pedro Pathing — Coordinates](https://pedropathing.com/docs/pathing/reference/coordinates).

Pedro Pathing uses a right-handed system that its own docs describe as
**"nonstandard to the FTC SDK Standard."**

### Origin
`(0, 0)` is at a **field corner** (the bottom-left of Pedro's field view), so
both axes span the interval **[0, 144] inches** rather than being centered on 0.

### Axes
| Axis | Positive sense |
|------|----------------|
| **X** | Increases toward the **right** of the field |
| **Y** | Increases toward the **top** ("up") of the field |

### Heading
Standard unit-circle convention, in **radians**:
| Heading | Robot faces |
|---------|-------------|
| `0` (0°) | Right (+X) |
| `π/2` (90°) | Up (+Y) |
| `π` (180°) | Left (−X) |
| `3π/2` (270°) | Down (−Y) |

**Positive rotation is counter-clockwise (CCW)**, like a unit circle.

---

## 3. FTC SDK vs. Pedro Pathing — key differences

| Property | FTC SDK (field) | Pedro Pathing |
|----------|-----------------|---------------|
| Origin | Field **center** | Field **corner** (bottom-left) |
| Axis range | ≈ **−72 … +72** in | **0 … 144** in |
| X positive | Right (from Red Wall) | Right of field |
| Y positive | Away from Red Wall | Up the field |
| Handedness | Right-handed | Right-handed |
| Heading positive | CCW about +Z | CCW (unit circle) |
| Angle units | Any (`Pose2D` is unit-tagged) | Radians |

### Converting between them
Because the only structural difference is the **origin shift** (center vs.
corner), a translation of **+72 in on both X and Y** moves a center-origin pose
into a corner-origin pose (this is the same `+72` trick Pedro documents for
RoadRunner poses, since RoadRunner also uses a field-centered origin):

```
x_pedro = x_ftc + 72
y_pedro = y_ftc + 72
heading_pedro = heading_ftc      // both CCW-positive; only axis labeling differs
```

Pedro also ships a programmatic converter for the general case:

```java
Pose ftcStandard = PoseConverter.pose2DToPose(ftcPose2d, InvertedFTCCoordinates.INSTANCE);
Pose pedro       = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
```

> **Note:** the simple `+72` mapping assumes the axes are oriented the same way
> and only the origin differs. Always verify against Pedro's `PoseConverter` for
> your season's field setup before trusting an autonomous path.

---

## Sources
- [FIRST Tech Challenge — Field Coordinate System Definition](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html)
- [Official FTC Field Coordinate System PDF](https://acmerobotics.github.io/ftc-dashboard/official_field_coord_sys.pdf)
- [Pedro Pathing — Coordinates](https://pedropathing.com/docs/pathing/reference/coordinates)