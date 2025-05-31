#ifndef ARM_CONFIG_HPP
#define ARM_CONFIG_HPP

// Lengtes van de links in mm (omgezet van meters in parse_urdf output)
#define LINK1_LENGTH 0.0f   // base_joint heeft geen lengte (fixed joint)
#define LINK2_LENGTH 113.0f // L2: 0.113 m → 113 mm
#define LINK3_LENGTH 180.0f // L3: 0.180 m → 180 mm
#define LINK4_LENGTH 43.5f  // L4: 0.0435 m → 43.5 mm
#define LINK5_LENGTH 176.0f // L5: 0.176 m → 176 mm
#define LINK6_LENGTH 0.0f   // L6: 0.000 m → 0 mm (waarschijnlijk end effector)

#define LINK1_WEIGHT 1.0f // Gewicht onbekend, laat zoals het was
#define LINK2_WEIGHT 1.0f
#define LINK3_WEIGHT 1.0f
#define LINK4_WEIGHT 1.0f
#define LINK5_WEIGHT 1.0f
#define LINK6_WEIGHT 1.0f

// Hoeklimieten van de joints in radialen en graden (uit parse_urdf.py output)
// Hier in graden, afgerond naar 2 decimalen:

#define JOINT1_MIN_ANGLE -97.40f
#define JOINT1_MAX_ANGLE 97.40f

#define JOINT2_MIN_ANGLE -56.15f
#define JOINT2_MAX_ANGLE 57.30f

#define JOINT3_MIN_ANGLE -114.59f
#define JOINT3_MAX_ANGLE 74.48f

#define JOINT4_MIN_ANGLE -114.59f
#define JOINT4_MAX_ANGLE 114.59f

#define JOINT5_MIN_ANGLE -120.32f
#define JOINT5_MAX_ANGLE 120.32f

#define JOINT6_MIN_ANGLE -177.62f
#define JOINT6_MAX_ANGLE 177.62f

// === JOINT ORIGINS, RPY AND AXES (uit URDF) ===
const Vector3D JOINT1_ORIGIN(0.0f, 0.0f, 0.0f);
const Vector3D JOINT1_RPY(0.0f, 0.0f, 0.0f);
const Vector3D JOINT1_AXIS(0.0f, 0.0f, 1.0f);

const Vector3D JOINT2_ORIGIN(0.02342f, 0.0f, 0.1105f);
const Vector3D JOINT2_RPY(-1.5708f, 0.0f, 0.0f);
const Vector3D JOINT2_AXIS(0.0f, 0.0f, 1.0f);

const Vector3D JOINT3_ORIGIN(0.0f, -0.18f, 0.0f);
const Vector3D JOINT3_RPY(3.1416f, 0.0f, -1.5708f);
// const Vector3D JOINT3_RPY(0.0f, 0.0f, 0.0f);
const Vector3D JOINT3_AXIS(0.0f, 0.0f, -1.0f);

const Vector3D JOINT4_ORIGIN(0.0435f, 0.0f, 0.0f);
const Vector3D JOINT4_RPY(1.5708f, 0.0f, 3.1416f);
// const Vector3D JOINT4_RPY(0.0f, 0.0f, 0.0f);
const Vector3D JOINT4_AXIS(0.0f, 0.0f, -1.0f);

const Vector3D JOINT5_ORIGIN(0.0f, 0.0f, -0.17635f);
const Vector3D JOINT5_RPY(-1.5708f, 0.0f, 0.0f);
const Vector3D JOINT5_AXIS(0.0f, 0.0f, -1.0f);

const Vector3D JOINT6_ORIGIN(0.0f, 0.0f, 0.0f);
const Vector3D JOINT6_RPY(1.5708f, 0.0f, 0.0f);
const Vector3D JOINT6_AXIS(0.0f, 0.0f, -1.0f);

#endif
