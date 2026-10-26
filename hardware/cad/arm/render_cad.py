"""Render the CAD assembly to PNG for the report (matplotlib, no display needed)."""
import sys, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import build_cad as B

def tris(wp):
    shape = wp.val() if hasattr(wp, "val") else wp
    vs, ts = shape.tessellate(0.5, 0.3)
    return [[(vs[i].x, vs[i].y, vs[i].z) for i in t] for t in ts]

def render(pose, fname, title, elev=18, azim=-60):
    fig = plt.figure(figsize=(7, 7), dpi=150)
    ax = fig.add_subplot(111, projection="3d")
    groups = [(B.base_plate(), "#c9a36f"), (B.hub(), "#e8e8e8"),
              (B.bracket().rotate((0, 0, 0), (0, 0, 1), pose[0]), "#dcdcdc")]
    cols = ["#f4f4f4", "#ececec", "#c9a36f", "#555555"]
    groups += list(zip(B.place(*pose), cols))
    for wp, c in groups:
        pc = Poly3DCollection(tris(wp), facecolor=c, edgecolor="#777777", linewidths=0.08)
        ax.add_collection3d(pc)
    ax.set_xlim(-200, 300); ax.set_ylim(-250, 250); ax.set_zlim(0, 500)
    ax.set_box_aspect((500, 500, 500))
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)"); ax.set_zlabel("z (mm)")
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(title, fontname="DejaVu Serif")
    fig.tight_layout()
    fig.savefig(fname)
    plt.close(fig)

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    render((0, 0, 0), os.path.join(out, "cad_rest_pose.png"), "Rest pose (all servos at 90)")
    render((80.0, 46.6, 11.0), os.path.join(out, "cad_point_B.png"), "Pose at Point B", azim=-120)
    render((-80.0, 17.2, 43.8), os.path.join(out, "cad_point_E.png"), "Pose above the button at Point E", azim=-20)
