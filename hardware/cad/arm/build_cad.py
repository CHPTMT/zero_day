"""Group 31 robotic arm: parametric CAD (CadQuery), exported as STEP.

Dimensions come from arm_controller.ino (links, pivot height, sensor offsets)
and from photos of the built arm (cross-sections, hub, bracket). Run:
    python build_cad.py
It writes one STEP per part and two assembly STEPs (rest pose and Point B).
Units: mm. Frame: x forward (the way the arm faces at rest), y to the arm's
left, z up, origin at the centre of the base plate top face... see BASE_TOP.
"""
import math
import cadquery as cq

# ---- dimensions (mm) -------------------------------------------------------
PLATE = (250, 250, 20)          # plywood base plate: x, y, thickness
HUB_D, HUB_H = 120, 60          # turntable hub on the plate
SHOULDER_Z = 130                # main arm pivot above the table (20 + 110)
L1, L2 = 225, 160               # main arm and second arm, pivot to pivot / to sensor board
MAIN_W, MAIN_T, MAIN_WALL = 50, 30, 5      # main arm box: side face width, thickness, wall
SEC_W, SEC_T, SEC_WALL = 40, 20, 4         # second arm box
END = 22                        # material past each pivot centre
PIN_D = 6                       # pivot hole
WIN_L, WIN_H = 22, 24           # second arm side windows (photos)
BRACKET_GAP, BRACKET_T = MAIN_T + 4, 5     # turret side plates either side of the main arm
PUSHER_L, PUSHER_S = 70, 8      # button pusher bar
HEAD = dict(cross=(20, 110, 6), sound_leg=(60, 10, 6), colour_leg=(90, 10, 6))
SG90 = (23, 12.2, 22.7)         # servo body
SG90_FLANGE = (32.2, 12.2, 2.5)


def servo():
    body = cq.Workplane("XY").box(*SG90)
    flange = cq.Workplane("XY").box(*SG90_FLANGE).translate((0, 0, SG90[2] / 2 - 4))
    horn = cq.Workplane("XY").circle(3).extrude(4).translate((SG90[0] / 2 - 6, 0, SG90[2] / 2))
    return body.union(flange).union(horn)


def base_plate():
    return cq.Workplane("XY").box(*PLATE).translate((0, 0, PLATE[2] / 2))


def hub():
    h = (cq.Workplane("XY").circle(HUB_D / 2).extrude(HUB_H)
         .faces(">Z").workplane().rect(SG90[0] + 2, SG90[1] + 2).cutBlind(-SG90[2]))
    return h.translate((0, 0, PLATE[2]))


def bracket():
    """Two turret side plates on the hub, holding the main arm pivot."""
    z0 = PLATE[2] + HUB_H
    h = SHOULDER_Z - z0 + 30
    plates = None
    for side in (+1, -1):
        p = (cq.Workplane("XZ").center(0, z0 + h / 2).rect(80, h).extrude(BRACKET_T / 2, both=True)
             .faces(">Y").workplane(centerOption="CenterOfBoundBox").center(0, 0))
        p = p.cut(cq.Workplane("XZ").center(0, SHOULDER_Z).circle(PIN_D / 2).extrude(50, both=True))
        p = p.translate((0, side * (BRACKET_GAP / 2 + BRACKET_T / 2), 0))
        plates = p if plates is None else plates.union(p)
    return plates


def box_link(length, w, t, wall, windows=()):
    """Hollow box link along +X from pivot (x=0) to pivot (x=length).
    w = side face width (z), t = thickness (y)."""
    L = length + 2 * END
    outer = cq.Workplane("XY").box(L, t, w).translate((length / 2, 0, 0))
    inner = cq.Workplane("XY").box(L + 2, t - 2 * wall, w - 2 * wall).translate((length / 2, 0, 0))
    link = outer.cut(inner)
    for x in (0, length):
        link = link.cut(cq.Workplane("XZ").center(x, 0).circle(PIN_D / 2).extrude(t, both=True))
    for f in windows:
        link = link.cut(cq.Workplane("XY").box(WIN_L, t + 2, WIN_H).translate((f * length, 0, 0)))
    return link


def main_arm():
    return box_link(L1, MAIN_W, MAIN_T, MAIN_WALL)


def second_arm():
    return box_link(L2, SEC_W, SEC_T, SEC_WALL, windows=(0.30, 0.52, 0.74))


def sensor_head():
    """Plywood crossbar at the arm end with the sound and colour sensor legs."""
    cx, cy, cz = HEAD["cross"]
    cross = cq.Workplane("XY").box(cx, cy, cz)
    snd = cq.Workplane("XY").box(*HEAD["sound_leg"]).translate((HEAD["sound_leg"][0] / 2, -45, 0))
    col = cq.Workplane("XY").box(*HEAD["colour_leg"]).translate((HEAD["colour_leg"][0] / 2, 45, 0))
    us = cq.Workplane("XY").box(10, 45, 20).translate((5, 0, 13))       # HC-SR04 board and cans
    return cross.union(snd).union(col).union(us)


def pusher():
    """70 mm bar hanging below the arm end; its tip is 90 mm below the arm axis."""
    return cq.Workplane("XY").box(PUSHER_S, PUSHER_S, PUSHER_L).translate((0, 0, -(20 + PUSHER_L / 2)))


def place(base_deg, main_deg, elbow_deg):
    """Return located parts for a pose given as relative joint angles (as in arm_controller)."""
    sh = cq.Location(cq.Vector(0, 0, SHOULDER_Z))
    phi = main_deg            # main arm from vertical, + forward
    psi = elbow_deg - main_deg  # second arm from horizontal
    # main arm: built along +X, rotate to point up then lean forward by phi
    main = main_arm().rotate((0, 0, 0), (0, 1, 0), -(90 - phi))
    ex, ez = L1 * math.sin(math.radians(phi)), L1 * math.cos(math.radians(phi))
    sec = second_arm().rotate((0, 0, 0), (0, 1, 0), -psi).translate((ex, 0, ez))
    hx = ex + L2 * math.cos(math.radians(psi)); hz = ez + L2 * math.sin(math.radians(psi))
    head = sensor_head().rotate((0, 0, 0), (0, 1, 0), -psi).translate((hx, 0, hz))
    push = pusher().rotate((0, 0, 0), (0, 1, 0), -psi).translate((hx, 0, hz))
    turn = lambda w: w.translate((0, 0, SHOULDER_Z)).rotate((0, 0, 0), (0, 0, 1), base_deg)
    return [turn(main), turn(sec), turn(head), turn(push)]


def assembly(name, pose):
    a = cq.Assembly(name=name)
    a.add(base_plate(), name="base_plate", color=cq.Color(0.80, 0.65, 0.45))
    a.add(hub(), name="hub", color=cq.Color(0.95, 0.95, 0.95))
    a.add(bracket().rotate((0, 0, 0), (0, 0, 1), pose[0]), name="bracket", color=cq.Color(0.95, 0.95, 0.95))
    s = servo().translate((0, 0, PLATE[2] + HUB_H - SG90[2] / 2))
    a.add(s, name="servo_base", color=cq.Color(0.2, 0.35, 0.8))
    for i, side in enumerate((+1, -1)):
        sv = servo().rotate((0, 0, 0), (1, 0, 0), 90).translate((0, side * (BRACKET_GAP / 2 + BRACKET_T + 7), SHOULDER_Z))
        a.add(sv.rotate((0, 0, 0), (0, 0, 1), pose[0]), name=f"servo_main_{'left' if side > 0 else 'right'}", color=cq.Color(0.2, 0.35, 0.8))
    names = ["main_arm", "second_arm", "sensor_head", "pusher"]
    cols = [cq.Color(0.98, 0.98, 0.98), cq.Color(0.98, 0.98, 0.98), cq.Color(0.80, 0.65, 0.45), cq.Color(0.3, 0.3, 0.3)]
    for n, w, c in zip(names, place(*pose), cols):
        a.add(w, name=n, color=c)
    return a


if __name__ == "__main__":
    import os
    out = os.path.dirname(os.path.abspath(__file__))
    parts = dict(base_plate=base_plate(), hub=hub(), bracket=bracket(), main_arm=main_arm(),
                 second_arm=second_arm(), sensor_head=sensor_head(), pusher=pusher(), servo_sg90=servo())
    for n, w in parts.items():
        cq.exporters.export(w, os.path.join(out, f"{n}.step"))
    assembly("arm_rest", (0, 0, 0)).save(os.path.join(out, "assembly_rest_pose.step"))
    assembly("arm_point_B", (80.0, 46.6, 11.0)).save(os.path.join(out, "assembly_point_B.step"))
    print("written:", sorted(f for f in os.listdir(out) if f.endswith(".step")))
