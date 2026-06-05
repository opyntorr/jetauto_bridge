#!/usr/bin/env python3
"""
evo_checkpoints.py — error de localizacion en CHECKPOINTS medidos (APE escaso, robot real sin GT).

Compara los checkpoints estimados (checkpoints.csv que escribe evo_logger.py al pulsar Enter) contra
las posiciones MEDIDAS por ti (gt_checkpoints.csv). Empareja por etiqueta e imprime error por punto +
RMSE. Con --to-tum emite gt_cp.tum / est_cp.tum (timestamp = indice del punto) para correr ademas:
    evo_ape tum gt_cp.tum est_cp.tum -a

Uso:
    python3 evo_checkpoints.py --est ~/evo_runs/run1/checkpoints.csv --gt ~/evo_runs/gt_checkpoints.csv
    python3 evo_checkpoints.py --est .../checkpoints.csv --gt .../gt_checkpoints.csv --to-tum

Formato gt_checkpoints.csv (lo creas tu, posiciones en el frame `map`, una linea por checkpoint):
    # label,x,y[,yaw_deg]      <- la cabecera con '#' es opcional; yaw_deg es opcional
    A,0.00,0.00,0
    B,1.50,0.20,90
    C,1.40,1.80,180

Las etiquetas deben COINCIDIR con las que marcaste en el logger (si solo pulsaste Enter, fueron
cp1, cp2, ... en orden).
"""

import argparse
import csv
import math
import os


def _read_est(path):
    """checkpoints.csv del logger -> {label: dict(x,y,yaw_deg,...)} en orden."""
    out = []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            out.append({
                'label': row['label'].strip(),
                'x': float(row['x']), 'y': float(row['y']),
                'yaw_deg': float(row.get('yaw_deg', 'nan') or 'nan'),
            })
    return out


def _read_gt(path):
    """gt_checkpoints.csv (label,x,y[,yaw_deg]) tolerante a cabecera '#'/espacios."""
    out = []
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) < 3:
                continue
            label = parts[0]
            if label.lower() in ('label', 'etiqueta'):
                continue
            yaw = float(parts[3]) if len(parts) >= 4 and parts[3] != '' else math.nan
            out.append({'label': label, 'x': float(parts[1]), 'y': float(parts[2]), 'yaw_deg': yaw})
    return out


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser(description="APE en checkpoints medidos (robot real).")
    ap.add_argument('--est', required=True, help="checkpoints.csv del evo_logger")
    ap.add_argument('--gt', required=True, help="gt_checkpoints.csv (posiciones medidas, frame map)")
    ap.add_argument('--to-tum', action='store_true', help="emite gt_cp.tum/est_cp.tum para evo_ape")
    args = ap.parse_args()

    est = _read_est(os.path.expanduser(args.est))
    gt = _read_gt(os.path.expanduser(args.gt))
    gt_by_label = {g['label']: g for g in gt}

    print(f"est: {len(est)} checkpoints | gt: {len(gt)} checkpoints")
    print(f"{'label':<10}{'err_xy[m]':>12}{'err_yaw[deg]':>14}")
    print('-' * 36)

    errs = []
    matched = []
    for e in est:
        g = gt_by_label.get(e['label'])
        if g is None:
            print(f"{e['label']:<10}{'(sin gt)':>12}")
            continue
        d = math.hypot(e['x'] - g['x'], e['y'] - g['y'])
        if not math.isnan(e['yaw_deg']) and not math.isnan(g['yaw_deg']):
            dyaw = abs(_wrap_deg(e['yaw_deg'] - g['yaw_deg']))
            dyaw_s = f"{dyaw:>14.2f}"
        else:
            dyaw_s = f"{'-':>14}"
        print(f"{e['label']:<10}{d:>12.3f}{dyaw_s}")
        errs.append(d)
        matched.append((e, g))

    if errs:
        rmse = math.sqrt(sum(d * d for d in errs) / len(errs))
        print('-' * 36)
        print(f"emparejados: {len(errs)} | RMSE_xy = {rmse:.3f} m | "
              f"max = {max(errs):.3f} m | mean = {sum(errs)/len(errs):.3f} m")
    else:
        print("No hubo checkpoints emparejados (revisa que las etiquetas coincidan).")

    if args.to_tum and matched:
        out_dir = os.path.dirname(os.path.expanduser(args.est)) or '.'
        gt_p = os.path.join(out_dir, 'gt_cp.tum')
        est_p = os.path.join(out_dir, 'est_cp.tum')
        with open(gt_p, 'w') as fg, open(est_p, 'w') as fe:
            for i, (e, g) in enumerate(matched):
                # timestamp = indice -> evo_ape asocia 1:1; z=0, orientacion identidad
                fg.write(f"{i:.6f} {g['x']:.6f} {g['y']:.6f} 0 0 0 0 1\n")
                fe.write(f"{i:.6f} {e['x']:.6f} {e['y']:.6f} 0 0 0 0 1\n")
        print(f"\nTUM escritos:\n  {gt_p}\n  {est_p}\n"
              f"Ahora: evo_ape tum {gt_p} {est_p} -a -p   (o --save_results rondaN.zip)")


if __name__ == '__main__':
    main()
