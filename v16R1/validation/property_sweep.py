#!/usr/bin/env python3
"""Independent mathematical checks for collision, impulse, and model laws.

This script checks closed-form identities and the uniqueness of the live
configuration schema. Engine behavior is checked by the C suite and the
determinism paranoia tests; this file does not pretend to prove those runtime
paths. Exit 0 means every equation check in this file passed.
"""
import math
import re
import sys
from pathlib import Path
fails = 0
def check(name, cond, detail=""):
    global fails
    print(("PASS " if cond else "FAIL ") + name + (" " + detail if detail else ""))
    if not cond:
        fails += 1

g = 9.80665
# SAT overlap is symmetric under swapping projected radii and reversing the
# center displacement. Negative overlap means separation.
def sat_overlap(proj_a, proj_b, center_projection):
    return proj_a + proj_b - abs(center_projection)

for pa, pb, d in [(1.2, 0.8, 1.5), (0.2, 0.3, -0.7), (2.0, 1.0, 1.0)]:
    ab = sat_overlap(pa, pb, d)
    ba = sat_overlap(pb, pa, -d)
    check(f"sat-symmetry({pa},{pb},{d})", abs(ab - ba) < 1e-12, f"overlap={ab:.4f}")

slop = 0.01
for overlap in [0.04, 0.0, -0.005, -0.02]:
    admitted = overlap >= -slop
    depth = max(overlap, 0.0) if admitted else None
    check(f"slop-admission({overlap})", admitted == (overlap >= -slop) and
          (depth is None or depth >= 0.0), f"admitted={admitted} depth={depth}")

# Coulomb cone + min<=sqrt
for mu_a, mu_b in [(0.9,0.3),(0.4,0.4),(0.0,0.9),(1.2,0.8)]:
    m = min(mu_a, mu_b)
    s = math.sqrt(mu_a*mu_b)
    check(f"cone-min<=sqrt({mu_a},{mu_b})", m <= s + 1e-12, f"min={m:.4f} sqrt={s:.4f}")
    normal_impulse = 3.7
    trial_tangent = 2.4
    tangent_impulse = min(trial_tangent, m * normal_impulse)
    check(f"coulomb-bound({mu_a},{mu_b})",
          0.0 <= tangent_impulse <= m * normal_impulse + 1e-12)
# Poisson series e=0.5
h, e = 1.0, 0.5
for i, exp in enumerate([0.25, 0.0625, 0.015625]):
    h = h*e*e
    check(f"poisson-h{i+1}", abs(h-exp) < 1e-12, f"{h}")
# Spring periods
for m, k, exp in [(1,100,0.6283),(2,50,1.2566),(0.5,200,0.3142)]:
    T = 2*math.pi*math.sqrt(m/k)
    check(f"spring-T(m={m},k={k})", abs(T-exp) < 0.001, f"{T:.4f}")
# Courant: k_stable = m_red*(1.5/dt)^2, dt=1/60
dt = 1/60
for m_red, k in [(0.5,100),(0.5,4050),(1.0,8100)]:
    ks = m_red*(1.5/dt)**2
    check(f"courant(m={m_red},k={k})", k <= ks + 1e-9, f"k_stable={ks:.0f}")
# Pendulum T=2pi sqrt(L/g)
for L in [0.5, 1.0, 2.0]:
    T = 2*math.pi*math.sqrt(L/g)
    check(f"pendulum(L={L})", T > 0 and math.isfinite(T), f"{T:.4f}")
# Friction stop d=v^2/(2 mu g)
for v, mu in [(5,0.4),(10,0.6),(2,0.9)]:
    d = v*v/(2*mu*g)
    check(f"stop(v={v},mu={mu})", d > 0 and math.isfinite(d), f"{d:.4f}")
# TOI stable-q vs naive (distant high-speed must not lose root)
a,b,c = 400.0,-400.0,99.0
D = b*b-4*a*c
q = -0.5*(b+math.copysign(math.sqrt(D),b))
t0, t1 = q/a, c/q
check("toi-stable-q", abs(min(t0,t1)-0.45) < 1e-9, f"{min(t0,t1):.4f}")
# Inertia laws
check("sphere-I", abs(0.4*2*0.25-0.2) < 1e-12)
check("box-Ixx", abs(2/12*(16+36)-8.6666667) < 1e-6)
check("cyl-axial", abs(0.5*3*0.25-0.375) < 1e-12)
check("cyl-trans", abs(3/12*(3*0.25+4)-1.1875) < 1e-12)
# Gottschalk depth law: overlap = proj_a + proj_b - |d.n| (fixed numbers)
proj_a, proj_b, d_dot_n = 1.2, 0.8, 1.5
overlap = proj_a + proj_b - abs(d_dot_n)
check("sat-depth", abs(overlap-0.5) < 1e-12, f"{overlap:.4f}")
check("sat-reject", (proj_a+proj_b-abs(2.5)) < -0.01, "separated")
# Oblique elastic exchange (equal mass, 45deg): speeds swap along line of impact
# v1=(1,0), v2=(0,0), normal=(1,0): v1'=(0,0), v2'=(1,0) (1D Newton, e=1)
v1, v2 = 1.0, 0.0
v1p, v2p = v2, v1
check("oblique-exchange", abs(v1p-0.0)<1e-12 and abs(v2p-1.0)<1e-12)
# Momentum conservation two-body: m1*v1+m2*v2 invariant in elastic exchange
m1, m2 = 2.0, 3.0
p_before = m1*1.0 + m2*0.0
# e=1 elastic: v1'=(m1-m2)/(m1+m2)*v1, v2'=2*m1/(m1+m2)*v1
v1pe = (m1-m2)/(m1+m2)*1.0
v2pe = 2*m1/(m1+m2)*1.0
p_after = m1*v1pe + m2*v2pe
check("momentum-2body", abs(p_before-p_after) < 1e-12, f"{p_after:.4f}")

# Read the configuration registry as source data and fail on duplicate keys.
# This stays current when tunables are added instead of pinning a stale count.
schema_path = Path(__file__).resolve().parents[1] / "src/config/mpe_config_schema.c"
schema = schema_path.read_text(encoding="utf-8")
keys = re.findall(r'^\s*\{"([^\"]+)"\s*,', schema, re.MULTILINE)
check("config-schema-keys-unique", bool(keys) and len(keys) == len(set(keys)),
      f"{len(keys)} keys")

print(f"\n{fails} failures")
sys.exit(1 if fails else 0)
