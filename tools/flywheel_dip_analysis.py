import sys
import pandas as pd, numpy as np

# Usage: python3 flywheel_dip_analysis.py <akit-export.csv>
CSV = sys.argv[1]

df = pd.read_csv(CSV, usecols=[
 "/Timestamp","/ShooterSubsystem/Flywheel/LeftMotorVelocity","/ShooterSubsystem/Flywheel/RightMotorVelocity",
 "/ShooterSubsystem/Flywheel/LeftMotorAppliedVoltage","/ShooterSubsystem/Flywheel/RightMotorAppliedVoltage",
 "/ShooterSubsystem/Flywheel/LeftMotorStatorCurrent","/ShooterSubsystem/Flywheel/RightMotorStatorCurrent"])
df = df.sort_values("/Timestamp").reset_index(drop=True)
t = df["/Timestamp"].values*1e-6
lv, rv = df["/ShooterSubsystem/Flywheel/LeftMotorVelocity"].values, df["/ShooterSubsystem/Flywheel/RightMotorVelocity"].values
lV, rV = df["/ShooterSubsystem/Flywheel/LeftMotorAppliedVoltage"].values, df["/ShooterSubsystem/Flywheel/RightMotorAppliedVoltage"].values
lI, rI = df["/ShooterSubsystem/Flywheel/LeftMotorStatorCurrent"].values, df["/ShooterSubsystem/Flywheel/RightMotorStatorCurrent"].values
v = 0.5*(lv+rv); I = 0.5*(lI+rI); V = 0.5*(lV+rV)
acc = np.gradient(v, t)
spin = (lv>60)&(rv>60)&np.isfinite(v)&np.isfinite(I)&np.isfinite(V)
print("n=%d, dt=%.1f ms, vmax=%.1f rad/s" % (len(t), 1000*np.median(np.diff(t)), np.nanmax(v)))

print("\n=== BALL-LOAD DIP EVENTS ===")
print(f"{'t(s)':>7} {'base':>7} {'min':>7} {'dip%':>6} {'dur(ms)':>7} {'rec(ms)':>7} {'I@base':>6} {'Ipk':>6} {'V@dip':>6}")
n=len(t)
base = np.full(n, np.nan)
for i in range(n):
    if spin[i]: base[i]=np.nanmedian(v[max(0,i-30):i+31])
dip = (v < base*0.985) & spin & (v>0)
events=[]
i=0
while i < n:
    if dip[i]:
        j=i
        while j+1<n and dip[j+1]: j+=1
        b=np.nanmedian(base[i:j+1]); m=np.nanmin(v[i:j+1])
        if (j-i+1)>=2 and m < b*0.99 and i>40:
            pre=np.nanmedian(base[max(0,i-40):i])
            k=j; rec=float('nan')
            while k < min(n, j+150):
                if v[k] > pre*0.995: rec=(t[k]-t[j])*1000; break
                k+=1
            ipk=int(np.nanargmax(I[i:j+1]))+i
            print(f"{t[i]:7.2f} {b:7.1f} {m:7.1f} {100*(m-b)/b:6.2f} {(t[j]-t[i])*1000:7.0f} {('-' if rec!=rec else f'{rec:.0f}'):>7} "
                  f"{np.nanmedian(I[max(0,i-30):i-1]):6.1f} {I[ipk]:6.1f} {V[ipk]:6.1f}")
            events.append((i,j,b,m))
        i=j+1
    else: i+=1
print("dip events:", len(events))

print("\n=== PLANT IDENTIFICATION ===")
ss = spin & (np.abs(acc)<2) & (np.abs(np.gradient(I,t))<1)
mss = ss & (v>150)
A=np.c_[np.ones(mss.sum()), v[mss]]
cv,*_=np.linalg.lstsq(A,V[mss],rcond=None)
ci,*_=np.linalg.lstsq(A,I[mss],rcond=None)
print("steady-state n=%d" % mss.sum())
for lo,hi in [(150,250),(250,350),(350,420),(420,470)]:
    m=mss&(v>=lo)&(v<hi)
    if m.sum()>20: print(f"  v {lo}-{hi}: V={np.nanmean(V[m]):5.2f}  I={np.nanmean(I[m]):5.2f} A")
print("V ~ %.3f + %.5f*w  => kv=%.4f V/(rad/s), ideal free-speed @12V=%.0f rad/s=%.0f rotor rpm" % (cv[0],cv[1],cv[1],12/cv[1],60*12/cv[1]/(2*np.pi)*2*np.pi/1))
print("I ~ %.2f + %.5f*w  => friction+load current %.1f A at 460 rad/s" % (ci[0],ci[1],ci[0]+ci[1]*460))

kt=0.00967
up = spin & (acc>3) & (v<430)
Amat=np.c_[I[up],np.ones(up.sum()),v[up]]
c,*_=np.linalg.lstsq(Amat,acc[up],rcond=None)
J=kt/c[0]; c0=c[1]*J; c1=c[2]*J
print("\nspin-up dyn fit (n=%d): dv/dt=%.5f*I - %.5f - %.7f*w" % (up.sum(),c[0],c[1],c[2]))
print("=> J_total=%.5f kg*m^2, coulomb friction c0=%.4f N*m, viscous c1=%.6f N*m/(rad/s)" % (J,c0,c1))

print("\n=== BALL DISTURBANCE TORQUE ===")
for (i,j,b,m) in events:
    k=int(np.nanargmin(v[i:j+1]))
    if not spin[k]: continue
    tau_net=J*acc[k]; tau_em=kt*I[k]; tau_f=c0*np.sign(v[k])+c1*v[k]
    tb=tau_em-tau_f-tau_net
    print(f"t={t[k]:7.2f}  dip={100*(m-b)/b:5.1f}%  at-min: acc={acc[k]:7.1f} I={I[k]:5.1f}A V={V[k]:5.1f}V  ball torque~{tb:5.2f} N*m")

m=ss&(v>420)
print("\nCruise @>420 rad/s: I=%.1f A (stator cap 120A), V=%.2f V (old cap 10V, new 12V)" % (np.nanmean(I[m]),np.nanmean(V[m])))
np.save("/tmp/opencode/plant.npy", np.array([J,c0,c1,kt]))
