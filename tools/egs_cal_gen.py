#!/usr/bin/env python3
"""
Generates src/cars/<car>/calibration_data.cpp from the UN52 config app calibration database
(egs_db.bin from https://github.com/rnd-ash/ultimate-nag52-config-app). No values are typed in by hand.

egs_db.bin is LZ4 (raw block) compressed bincode (legacy config) of CalibrationDatabase,
see backend/src/diag/calibration.rs and config_app/src/ui/configuration/egs_config.rs in the config app.

Usage:
  List what is in the database for a chassis (and/or gearbox):
    python3 tools/egs_cal_gen.py egs_db.bin --list --chassis 210.026
    python3 tools/egs_cal_gen.py egs_db.bin --list --gearbox 722.626
  Generate the calibration for one EGS part number + chassis (+ gearbox if the pair has more than one)
  into the folder of a car (src/cars/<car>/, one PlatformIO environment per car, see platformio.ini):
    python3 tools/egs_cal_gen.py egs_db.bin --egs 0305452032 --chassis 210.026 --car w210_026
"""
import argparse
import os
import struct
import sys


def lz4_block_decompress(src):
    out = bytearray()
    i = 0
    n = len(src)
    while i < n:
        tok = src[i]
        i += 1
        lit = tok >> 4
        if lit == 15:
            while True:
                b = src[i]
                i += 1
                lit += b
                if b != 255:
                    break
        out += src[i:i + lit]
        i += lit
        if i >= n:
            break
        off = src[i] | (src[i + 1] << 8)
        i += 2
        ml = tok & 15
        if ml == 15:
            while True:
                b = src[i]
                i += 1
                ml += b
                if b != 255:
                    break
        ml += 4
        start = len(out) - off
        for k in range(ml):
            out.append(out[start + k])
    return bytes(out)


class Reader:
    def __init__(self, b):
        self.b = b
        self.i = 0

    def u8(self):
        v = self.b[self.i]
        self.i += 1
        return v

    def u16(self):
        v = struct.unpack_from('<H', self.b, self.i)[0]
        self.i += 2
        return v

    def i16(self):
        v = struct.unpack_from('<h', self.b, self.i)[0]
        self.i += 2
        return v

    def u64(self):
        v = struct.unpack_from('<Q', self.b, self.i)[0]
        self.i += 8
        return v

    def string(self):
        n = self.u64()
        v = self.b[self.i:self.i + n].decode()
        self.i += n
        return v

    def vec(self, f):
        return [f() for _ in range(self.u64())]


# Field order and names of the UN52 C structs (src/egs_calibration/calibration_structs.h),
# which have the same layout as the config app's Rust structs
HYDR_FIELDS = [
    ('p_multi_1', 'u16', 1), ('p_multi_other', 'u16', 1), ('lp_reg_spring_pressure', 'u16', 1),
    ('overlap_circuit_factor_spc', 'u16', 8), ('overlap_circuit_factor_mpc', 'u16', 8),
    ('overlap_circuit_spring_pressure', 'i16', 8), ('shift_reg_spring_pressure', 'u16', 1),
    ('shift_spc_gain', 'u16', 8), ('min_mpc_pressure', 'u16', 1), ('filter_factor', 'u8', 1),
    ('mpc_flush_temp_threshold', 'u8', 1), ('mpc_no_flush_time', 'u16', 1), ('mpc_flush_time', 'u16', 1),
    ('extra_p_not_shifting', 'u16', 1), ('shift_pressure_addr_percent', 'u16', 1),
    ('inlet_pressure_offset', 'u16', 1), ('inlet_pressure_input_min', 'u16', 1),
    ('inlet_pressure_input_max', 'u16', 1), ('inlet_pressure_output_min', 'u16', 1),
    ('inlet_pressure_output_max', 'u16', 1), ('extra_pressure_pump_speed_min', 'u16', 1),
    ('extra_pressure_pump_speed_max', 'u16', 1), ('extra_pressure_adder_r1_1', 'u16', 1),
    ('extra_pressure_adder_other_gears', 'u16', 1), ('shift_pressure_factor_percent', 'u16', 1),
    ('pcs_map_x', 'u16', 7), ('pcs_map_y', 'u16', 4), ('pcs_map_z', 'u16', 28),
]
MECH_FIELDS = [
    ('gb_ty', 'u8', 1), ('ratio_table', 'u16', 8), ('intertia_factor', 'u16', 8), ('friction_map', 'u16', 48),
    ('max_torque_on_clutch', 'u16', 4), ('max_torque_off_clutch', 'u16', 4), ('release_spring_pressure', 'u16', 6),
    ('intertia_torque', 'u16', 8), ('strongest_loaded_clutch_idx', 'u8', 8), ('turbine_drag', 'u16', 8),
    ('atf_density_minus_50c', 'u16', 1), ('atf_density_drop_per_c', 'u16', 1),
    ('atf_density_centrifugal_force_factor', 'u16', 3),
]
TCC_FIELDS = [
    ('multiplier_map_x', 'u16', 2), ('multiplier_map_z', 'u16', 2), ('pump_map_x', 'u16', 11), ('pump_map_z', 'u16', 11),
]
SHIFT_ALGO_FIELDS = [
    ('momentum_1_2_x', 'u8', 3), ('momentum_2_3_x', 'u8', 3), ('momentum_3_4_x', 'u8', 3), ('momentum_4_5_x', 'u8', 3),
    ('momentum_1_2_y', 'u8', 2), ('momentum_2_3_y', 'u8', 2), ('momentum_3_4_y', 'u8', 2), ('momentum_4_5_y', 'u8', 2),
    ('momentum_1_2_z', 'u8', 6), ('momentum_2_3_z', 'u8', 6), ('momentum_3_4_z', 'u8', 6), ('momentum_4_5_z', 'u8', 6),
    ('momentum_2_1_x', 'u8', 6), ('momentum_3_2_x', 'u8', 6), ('momentum_4_3_x', 'u8', 6), ('momentum_5_4_x', 'u8', 6),
    ('momentum_2_1_y', 'u8', 10), ('momentum_3_2_y', 'u8', 10), ('momentum_4_3_y', 'u8', 10), ('momentum_5_4_y', 'u8', 10),
    ('momentum_2_1_z', 'u8', 60), ('momentum_3_2_z', 'u8', 60), ('momentum_4_3_z', 'u8', 60), ('momentum_5_4_z', 'u8', 60),
    ('trq_adder_1_2_x', 'u8', 6), ('trq_adder_2_3_x', 'u8', 6), ('trq_adder_3_4_x', 'u8', 6), ('trq_adder_4_5_x', 'u8', 6),
    ('trq_adder_1_2_y', 'u8', 8), ('trq_adder_2_3_y', 'u8', 8), ('trq_adder_3_4_y', 'u8', 8), ('trq_adder_4_5_y', 'u8', 8),
    ('trq_adder_1_2_z', 'u8', 48), ('trq_adder_2_3_z', 'u8', 48), ('trq_adder_3_4_z', 'u8', 48), ('trq_adder_4_5_z', 'u8', 48),
    ('trq_adder_2_1_x', 'u8', 3), ('trq_adder_3_2_x', 'u8', 3), ('trq_adder_4_3_x', 'u8', 3), ('trq_adder_5_4_x', 'u8', 3),
    ('trq_adder_2_1_y', 'u8', 4), ('trq_adder_3_2_y', 'u8', 4), ('trq_adder_4_3_y', 'u8', 4), ('trq_adder_5_4_y', 'u8', 4),
    ('trq_adder_2_1_z', 'u8', 12), ('trq_adder_3_2_z', 'u8', 12), ('trq_adder_4_3_z', 'u8', 12), ('trq_adder_5_4_z', 'u8', 12),
]
# sizeof(ShiftAlgorithmPack), also checked with a static_assert in the generated file
SHIFT_ALGO_LEN = 672
assert sum(n for _, _, n in SHIFT_ALGO_FIELDS) == SHIFT_ALGO_LEN


def read_fields(r, fields):
    out = []
    for name, ty, count in fields:
        f = getattr(r, ty)
        out.append((name, ty, f() if count == 1 else [f() for _ in range(count)]))
    return out


def load_db(path):
    with open(path, 'rb') as f:
        r = Reader(lz4_block_decompress(f.read()))

    def chassis():
        keys = ['gearbox', 'chassis', 'hydr_cfg', 'mech_cfg', 'tcc_cfg', 'shift_algo_cfg']
        return {k: r.string() for k in keys}

    def record(fields):
        def f():
            name = r.string()
            data = read_fields(r, fields)
            pns = r.vec(r.string)
            return {'name': name, 'data': data, 'pns': pns}
        return f

    db = {}
    db['egs'] = r.vec(lambda: {'pn': r.string(), 'chassis': r.vec(chassis)})
    db['hydr'] = r.vec(record(HYDR_FIELDS))
    db['mech'] = r.vec(record(MECH_FIELDS))
    db['tcc'] = r.vec(record(TCC_FIELDS))
    db['shift'] = r.vec(record(SHIFT_ALGO_FIELDS))
    if r.i != len(r.b):
        sys.exit('egs_db.bin was not fully read (%d of %d bytes), the format has changed' % (r.i, len(r.b)))
    return db


def find_record(records, name, pn, what):
    # Names are not unique in the database. The config app takes the FIRST record with this
    # name that lists the EGS part number (egs_config.rs, .iter().find()), so do exactly the same
    hits = [x for x in records if x['name'] == name and pn in x['pns']]
    if not hits:
        sys.exit('No %s calibration named %r for EGS %s' % (what, name, pn))
    if any(h['data'] != hits[0]['data'] for h in hits[1:]):
        print('NOTE: %d different %s calibrations are named %r for EGS %s, using the first one like the config app does'
              % (len(hits), what, name, pn))
    return hits[0]


def c_value(ty, v):
    if isinstance(v, list):
        return '{' + ', '.join(str(x) for x in v) + '}'
    return str(v)


def c_struct(ty_name, var, rec, flash=False):
    # flash: 'const ... PROGMEM', the AVR keeps it in flash instead of copying it to RAM
    if flash:
        lines = ['extern const %s %s; // const alone would be internal to this file' % (ty_name, var),
                 'const %s %s PROGMEM = {' % (ty_name, var)]
    else:
        lines = ['%s %s = {' % (ty_name, var)]
    for name, ty, v in rec['data']:
        lines.append('    %s, // %s' % (c_value(ty, v), name))
    lines.append('};')
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('egs_db')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--egs', help='EGS part number, eg 0305452032')
    ap.add_argument('--chassis', help='eg 210.026')
    ap.add_argument('--gearbox', help='eg 722.626')
    ap.add_argument('--car', help='car folder in src/cars, eg w124_210_016. Writes src/cars/<car>/calibration_data.cpp')
    ap.add_argument('--out', help='output file, instead of --car')
    a = ap.parse_args()
    if a.car and not a.out:
        a.out = os.path.join(os.path.dirname(__file__), '..', 'src', 'cars', a.car, 'calibration_data.cpp')
    db = load_db(a.egs_db)

    rows = []
    for egs in db['egs']:
        for c in egs['chassis']:
            if a.egs and egs['pn'] != a.egs:
                continue
            if a.chassis and not c['chassis'].startswith(a.chassis):
                continue
            if a.gearbox and not c['gearbox'].startswith(a.gearbox):
                continue
            row = (egs['pn'], c)
            if row not in rows:  # The database has some entries twice
                rows.append(row)

    if a.list or not a.egs:
        for pn, c in rows:
            print('EGS %s  chassis %s  gearbox %-16s hydr %-5s mech %-5s tcc %-5s shift %s' % (
                pn, c['chassis'], c['gearbox'], c['hydr_cfg'], c['mech_cfg'], c['tcc_cfg'], c['shift_algo_cfg']))
        if not rows:
            print('Nothing found')
        return

    if not a.out:
        sys.exit('Give --car (or --out) to say where the calibration goes')
    if len(rows) != 1:
        sys.exit('%d entries match, narrow it down with --chassis / --gearbox (use --list)' % len(rows))
    pn, c = rows[0]
    hydr = find_record(db['hydr'], c['hydr_cfg'], pn, 'hydraulic')
    mech = find_record(db['mech'], c['mech_cfg'], pn, 'mechanical')
    tcc = find_record(db['tcc'], c['tcc_cfg'], pn, 'torque converter')
    shift = find_record(db['shift'], c['shift_algo_cfg'], pn, 'shift algorithm')

    src = '''// GENERATED by tools/egs_cal_gen.py, do not edit by hand. Re-run the script to change it.
//
// Source: UN52 config app egs_db.bin
// EGS: %s, chassis: %s, gearbox: %s
// Hydraulic: %s, mechanical: %s, torque converter: %s, shift algorithm: %s
#include "../../egs_calibration/calibration_structs.h"
#include <avr/pgmspace.h>

static_assert(sizeof(ShiftAlgorithmPack) == %d, "ShiftAlgorithmPack layout does not match egs_db.bin");

extern const bool CAL_GENERATED;
const bool CAL_GENERATED = true;
extern const char CAL_DESCRIPTION[];
const char CAL_DESCRIPTION[] PROGMEM = "EGS %s %s %s hydr %s mech %s tcc %s shift %s";

%s

%s

%s

%s
''' % (pn, c['chassis'], c['gearbox'], hydr['name'], mech['name'], tcc['name'], shift['name'], SHIFT_ALGO_LEN,
       pn, c['chassis'], c['gearbox'], hydr['name'], mech['name'], tcc['name'], shift['name'],
       c_struct('HydraulicCalibration', 'HYDR_CAL', hydr),
       c_struct('MechanicalCalibration', 'MECH_CAL', mech),
       c_struct('TorqueConverterCalibration', 'TCC_CAL', tcc),
       c_struct('ShiftAlgorithmPack', 'SHIFT_ALGO_CAL', shift, flash=True))
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w') as f:
        f.write(src)
    print('Wrote %s (%s)' % (os.path.normpath(a.out), 'EGS %s %s %s' % (pn, c['chassis'], c['gearbox'])))


if __name__ == '__main__':
    main()
