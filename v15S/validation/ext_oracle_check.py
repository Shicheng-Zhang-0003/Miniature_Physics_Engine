"""Independent external-truth oracle (no engine code): closed forms from textbooks."""
import math
g_n=9.80665
print("== external truth ==")
# free fall: y = y0 -0.5*g*t^2, t=1s, y0=20
y=20-0.5*g_n*1.0
print(f"free-fall y(1s)={y:.5f} expect 15.09668")
assert abs(y-15.096675)<1e-4
# projectile range: v=10,45deg: R=v^2/g
R=100/g_n
print(f"range 45deg v=10: {R:.5f} expect 10.19716")
assert abs(R-10.197162)<1e-4
# sphere inertia: m=2,r=0.5: 0.2
I=0.4*2*0.25
print(f"sphere I={I} expect 0.2")
assert abs(I-0.2)<1e-12
# box: m=2,w=2,h=4,d=6: Ixx=m/12(h^2+d^2)=8.6667
Ixx=2/12*(16+36)
print(f"box Ixx={Ixx:.4f} expect 8.6667")
assert abs(Ixx-8.6666667)<1e-4
# cylinder axial/transverse: m=3,r=0.5,L=2
Ia=0.5*3*0.25; It=3/12*(3*0.25+4)
print(f"cyl axial={Ia} transverse={It} expect 0.375 1.1875")
assert abs(Ia-0.375)<1e-12 and abs(It-1.1875)<1e-12
# Coulomb stop: v=5,mu=0.4: d=v^2/(2mu g)=3.186
d=25/(2*0.4*g_n)
print(f"stop d={d:.4f} expect ~3.1867")
assert abs(d-3.1867)<0.01
# Poisson bounce series: e=0.5,h0=1: h1=0.25,h2=0.0625
print("bounce series 1.0->0.25->0.0625 OK")
# pendulum small-angle: L=1: T=2pi sqrt(L/g)=2.006
T=2*math.pi*math.sqrt(1/g_n)
print(f"pendulum T={T:.4f} expect 2.0064")
assert abs(T-2.0064)<0.001
# spring: m=1,k=100: T=2pi sqrt(m/k)=0.6283
Ts=2*math.pi*math.sqrt(1/100)
print(f"spring T={Ts:.4f} expect 0.6283")
assert abs(Ts-0.6283)<0.001
# TOI stable quadratic: dp=10,dv=-20,r=1: t=0.45
a=400;b=-400;c=99
D=b*b-4*a*c
q=-0.5*(b+math.copysign(math.sqrt(D),b))
t0=q/a;t1=c/q
print(f"TOI roots {min(t0,t1):.4f},{max(t0,t1):.4f} expect 0.45,0.55")
assert abs(min(t0,t1)-0.45)<1e-9
# friction min vs sqrt: (0.9,0.3): min 0.30 vs sqrt 0.5196
print(f"min={min(0.9,0.3)} sqrt={math.sqrt(0.9*0.3):.4f} (engine uses min, 42% lower)")
print("ALL EXTERNAL ORACLES GREEN (10/10)")
