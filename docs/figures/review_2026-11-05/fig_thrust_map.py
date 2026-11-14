#!/usr/bin/env python3
"""The thrust map (3.3.2; Wesley 8 Nov: a figure for the thrust map).

The one sentence: hover throttle grows affinely with the mass carried, with an offset of about 0.185 at zero mass, so a
proportional map identified at one hover point over-predicts the throttle once a mass is hung; the battery voltage
shifts the offset by about 0.022 per volt.
Hero: (a) the free hovers' throttle, corrected to the reference voltage, against the total mass (quadrotor as flown plus
the hung mass), with the affine fit (blue) and the proportional map through the unloaded hover point (red). (b) the
throttle corrected to the unloaded mass against the pack voltage: the affine fit slopes, the proportional map is flat.
8 Nov (Wesley): x from 0.5 kg, no offset label, no dashed extension or band, no labels on the lines (the legend names
them), the y labels as '(corrected to ...)'; the fit's numbers and errors go in the text.
Left out: which quadrotor each point is (the offsets differ by 0.006: in the text's fit), the tethered check.
Data: RIG-0930-ladder1 / ladder4, 13 free hovers (tools/test/fixtures/rig_0930/ladder_points.yaml), fitted exactly as
tools/thrust_fit.py fits them (steady_hover, fit).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import review_style as S  # noqa: E402
sys.path.insert(0, S.TOOLS)
import thrust_fit as TF  # noqa: E402

S.HERE = HERE
SPEC = os.path.join(S.TOOLS, 'test', 'fixtures', 'rig_0930', 'ladder_points.yaml')


def main():
    law = TF.fit(TF.read_spec(SPEC))
    rows = law['rows']
    a_mean = float(np.nanmean([a for a in law['a'] if a == a]))
    print(f"n {law['n']}, a {[round(a, 3) for a in law['a']]}, b {law['b']:.3f}, c {law['c']:.4f}, "
          f"rms {law['rms']:.4f}, max {law['max']:.4f}")
    m = np.array([r['m'] for r in rows])
    u = np.array([r['u'] for r in rows])
    v = np.array([r['v'] for r in rows])
    off = np.array([law['a'][r['drone']] for r in rows])
    u_ref = u - law['c'] * (v - TF.V_REF)                  # throttle at the reference voltage
    u_m = u_ref - off + a_mean                             # and on the mean offset, for one fitted line
    # the proportional map: through zero and the unloaded hovers' mean (the map identified at hover)
    unl = np.isclose(m, TF.DRONE_KG)
    k_prop = float(np.mean(u_m[unl]) / TF.DRONE_KG)
    print(f"proportional: {k_prop:.3f} per kg = {TF.G / k_prop:.1f} N per unit throttle; "
          f"at {m.max():.3f} kg it predicts {k_prop * m.max():.3f} against the affine {a_mean + law['b'] * m.max():.3f}")

    S.setup()
    fig = S.figure('full', 2.5)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.35, 1.0], wspace=0.16)
    a, b = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])

    mm = np.linspace(0.5, 0.8, 20)
    line_aff, = a.plot(mm, a_mean + law['b'] * mm, color=S.BLUE, lw=S.LW_HERO)
    line_prop, = a.plot(mm, k_prop * mm, color=S.RED, lw=S.LW_HERO, ls=(0, (5, 1.5, 1, 1.5)))
    pts, = a.plot(m, u_m, 'o', ms=S.MS, color=S.INK, mec='white', mew=0.4, zorder=3)
    a.set_xlim(0.5, 0.8)
    a.set_ylim(0.4, 0.7)
    a.set_xticks([0.5, 0.6, 0.7, 0.8])
    a.set_xticklabels(['0.5', '0.6', '0.7', '0.8'])
    a.set_yticks([0.4, 0.5, 0.6, 0.7])
    a.set_yticklabels(['0.4', '0.5', '0.6', '0.7'])
    a.set_xlabel('Total mass carried by the quadrotor (kg)')
    a.set_ylabel(f'Hover throttle\n(corrected to {TF.V_REF:g}\\,V)')

    m0 = TF.DRONE_KG
    u_v = u - off + a_mean - law['b'] * (m - m0)           # throttle at the unloaded mass, on the mean offset
    vv = np.linspace(22.5, 24.6, 20)
    b.plot(vv, a_mean + law['b'] * m0 + law['c'] * (vv - TF.V_REF), color=S.BLUE, lw=S.LW_HERO)
    b.plot(vv, np.full_like(vv, k_prop * m0), color=S.RED, lw=S.LW_HERO, ls=(0, (5, 1.5, 1, 1.5)))
    b.plot(v, u_v, 'o', ms=S.MS, color=S.INK, mec='white', mew=0.4, zorder=3)
    b.set_xlim(22.5, 24.6)
    b.set_ylim(0.43, 0.50)
    b.set_yticks([0.44, 0.46, 0.48, 0.50])
    b.set_yticklabels(['0.44', '0.46', '0.48', '0.50'])
    b.set_xlabel('Battery voltage (V)')
    b.set_ylabel(f'Hover throttle\n(corrected to {m0:g}\\,kg)')
    S.panel(a, 'a', 'Against the carried mass')
    S.panel(b, 'b', 'Against the battery voltage')
    fig.align_ylabels((a, b))
    S.ps.legend_below(fig, [line_aff, line_prop], ['Affine fit', 'Proportional map'], ncol=2)
    S.save(fig, 'fig_thrust_map')


if __name__ == '__main__':
    main()
