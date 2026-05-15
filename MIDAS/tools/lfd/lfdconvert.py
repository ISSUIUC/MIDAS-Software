"""MIDAS .launch v2 file decoder

Usage:
    python lfdconvert.py <file.launch> [-o prefix]

Michael Karpov, 2026
"""
import argparse, csv, struct
from pathlib import Path


T_STRUCT, T_FLOAT32, T_FLOAT64 = 0, 1, 2
T_INT8, T_UINT8, T_INT16, T_UINT16 = 3, 4, 5, 6
T_INT32, T_UINT32, T_INT64, T_UINT64 = 7, 8, 9, 10
T_BOOL, T_ENUM, T_ARRAY, T_UNION = 11, 12, 13, 14

LEAVES = {
    T_FLOAT32: ("<f", 4), 
    T_FLOAT64: ("<d", 8),
    T_INT8: ("<b", 1), 
    T_UINT8: ("<B", 1),
    T_INT16: ("<h", 2),   
    T_UINT16: ("<H", 2),
    T_INT32: ("<i", 4),   
    T_UINT32: ("<I", 4),
    T_INT64: ("<q", 8),   
    T_UINT64: ("<Q", 8),
    T_BOOL: ("<?", 1),   
    T_ENUM: ("<i", 4),
}

# struct LogFormatMetaEntry { char[32]; uint8_t; uint16_t; } 
META_ENTRY = struct.Struct("<32sBxH")
# struct LogDiscMapEntry { uint16_t disc; uint16_t entry_index; }
DISC_ENTRY = struct.Struct("<HH")

META_CODES = {
    0: ("EVENT_TLAUNCH", T_UINT32),
    1: ("EVENT_TBURNOUT", T_UINT32),
    2: ("EVENT_TIGNITION", T_UINT32),
    3: ("EVENT_TAPOGEE", T_UINT32),
    4: ("EVENT_TMAIN", T_UINT32),
    5: ("EVENT_TMAX_ACCEL", T_UINT32),
    6: ("EVENT_TMAX_VEL", T_UINT32),
    7: ("EVENT_TMAX_DESCENT_RATE", T_UINT32),
    8: ("DATA_LAUNCHSITE_BARO", T_FLOAT32),
    9: ("DATA_LAUNCHSITE_ALT", T_FLOAT32),
    10: ("DATA_LAUNCHSITE_GPS_ALT", T_UINT32),
    11: ("DATA_LAUNCHSITE_GPS_LAT", T_UINT32),
    12: ("DATA_LAUNCHSITE_GPS_LONG", T_UINT32),
    13: ("DATA_LAUNCH_INITIAL_TILT", T_FLOAT32),
    14: ("DATA_TILT_AT_BURNOUT", T_FLOAT32),
    15: ("DATA_TILT_AT_IGNITION", T_FLOAT32),
    16: ("DATA_BARO_AT_IGNITION", T_FLOAT32),
    17: ("DATA_MAX_ACCEL", T_FLOAT32),
    18: ("DATA_MAX_VEL", T_FLOAT32),
    19: ("DATA_ALT_AT_BURNOUT", T_FLOAT32),
    20: ("DATA_MAX_DESCENT_RATE", T_FLOAT32),
}


def align_up(off, align):
    return off + (-off) % align

def fmt_value(type_id, raw):
    """Format one leaf value for CSV output."""
    val = struct.unpack(LEAVES[type_id][0], raw)[0]
    if type_id == T_FLOAT32: return format(val, ".9g")
    if type_id == T_FLOAT64: return format(val, ".17g")
    if type_id == T_BOOL:    return "1" if val else "0"
    return str(val)

class Node:
    def __init__(self, name, type_id, count):
        self.name = name
        self.type_id = type_id
        self.count = count
        self.children = []
        self.size = 0
        self.align = 1


def build_tree(entries):
    # Rebuilds the tree from the preorder traversal
    idx_to_node = {}
    pos = 0

    def build():
        nonlocal pos
        name, tid, count = entries[pos]
        n = Node(name, tid, count)
        idx_to_node[pos] = n
        pos += 1

        if tid in (T_STRUCT, T_UNION):
            for _ in range(count):
                n.children.append(build())
        elif tid == T_ARRAY:
            n.children.append(build())

        if tid == T_STRUCT:
            off = 0
            for c in n.children:
                off = align_up(off, c.align) + c.size
                n.align = max(n.align, c.align)
            n.size = align_up(off, n.align)
        elif tid == T_UNION:
            n.size = max((c.size for c in n.children), default=0)
            n.align = max((c.align for c in n.children), default=1)
        elif tid == T_ARRAY:
            c = n.children[0]
            n.size = align_up(c.size, c.align) * count
            n.align = c.align
        else:
            n.size = n.align = LEAVES[tid][1]
        return n

    root = build()
    return root, idx_to_node


def leaves_of(node):
    out = []
    def walk(n, prefix, base):
        if n.type_id == T_STRUCT:
            off = 0
            for c in n.children:
                off = align_up(off, c.align)
                child_prefix = c.name if not prefix else f"{prefix}.{c.name}"
                walk(c, child_prefix, base + off)
                off += c.size
        elif n.type_id == T_ARRAY:
            c = n.children[0]
            stride = align_up(c.size, c.align)
            for i in range(n.count):
                walk(c, f"{prefix}[{i}]", base + i * stride)
        else:
            out.append((prefix, n.type_id, base))
    walk(node, "", 0)
    return out

def read_launch(path):
    blob = path.read_bytes()
    pos = 0

    def take(n):
        nonlocal pos
        if pos + n > len(blob):
            raise ValueError(f"unexpected EOF at offset {pos}")
        out = blob[pos:pos+n]; pos += n; return out
    def expect(lit):
        got = take(len(lit))
        if got != lit: raise ValueError(f"expected {lit}, got {got}")
    def u32():
        return struct.unpack("<I", take(4))[0]
    def line():
        nonlocal pos
        end = blob.index(b"\n", pos)
        out = blob[pos:end]; pos = end + 1; return out

    expect(b"LAUNCH "); version = u32(); expect(b"\n")
    expect(b"FILE "); name = line().decode("ascii", errors="replace")
    expect(b"CHECKSUM "); log_csum = u32(); expect(b" "); eep_csum = u32(); expect(b"\n")
    expect(b"META "); meta_sz = u32(); expect(b"\n")
    expect(b"BIN "); bin_sz = u32(); expect(b"\n")

    hdr = {"version": version, "name": name, "log_csum": log_csum, "eep_csum": eep_csum}
    return hdr, take(meta_sz), take(bin_sz)

def read_meta(blob):
    pos = 0

    def take(n):
        nonlocal pos
        out = blob[pos:pos+n]; pos += n; return out
    def chunk(n):
        nonlocal pos
        out = take(n)
        if blob[pos:pos+1] != b"\n":
            raise ValueError(f"missing newline at {pos}")
        pos += 1
        return out
    def u32(): return struct.unpack("<I", chunk(4))[0]
    def parse_tree():
        node_size = u32()
        count = u32()
        return count, chunk(node_size * count)

    u32()                          
    eeprom_data = chunk(u32())  

    log_count, log_bytes = parse_tree()
    log_entries = unpack_meta_entries(log_bytes, log_count)

    disc_count, disc_bytes = parse_tree()
    if len(disc_bytes) != DISC_ENTRY.size * disc_count:
        raise ValueError("LOG_DISCMAP mismatch size")
    disc_entries = [DISC_ENTRY.unpack_from(disc_bytes, i * DISC_ENTRY.size) for i in range(disc_count)]

    eep_count, eep_bytes = parse_tree()
    eeprom_entries = unpack_meta_entries(eep_bytes, eep_count)

    return log_entries, disc_entries, eeprom_data, eeprom_entries, blob[pos:]


def unpack_meta_entries(blob, count):
    if len(blob) != META_ENTRY.size * count:
        raise ValueError("LogFormatMetaEntry size mismatch")
    out = []
    for i in range(count):
        ident, tid, c = META_ENTRY.unpack_from(blob, i * META_ENTRY.size)
        out.append((ident.split(b"\x00", 1)[0].decode(), tid, c))
    return out

def write_flight_csv(bin_blob, expected_csum, log_entries, disc_entries, path):
    root, idx_to_node = build_tree(log_entries)
    variants = {disc: (idx_to_node[idx].name, idx_to_node[idx].size, leaves_of(idx_to_node[idx])) for disc, idx in disc_entries}
    columns = [(disc, variants[disc][0], leaf) for disc, _ in disc_entries for leaf in variants[disc][2]]

    bin_csum = struct.unpack("<I", bin_blob[:4])[0]
    if bin_csum != expected_csum:
        print(f"warning: checksum 0x{bin_csum:08x} !=  0x{expected_csum:08x}\n")

    pos = 4
    decoded_rows = []
    while pos + 8 <= len(bin_blob):
        disc, ts = struct.unpack("<II", bin_blob[pos:pos+8]); pos += 8
        v = variants.get(disc)
        if v is None:
            print(f"unknown discriminant {disc} at row {len(decoded_rows)}; aborting\n")
            break
        vname, vsize, _ = v
        if pos + vsize > len(bin_blob):
            print(f"missing data for {vname}\n")
            break
        data = bin_blob[pos:pos+vsize]; pos += vsize

        row = [str(ts), vname] + [""] * len(columns)
        for i, (col_disc, _, (_, tid, off)) in enumerate(columns):
            if col_disc == disc:
                sz = LEAVES[tid][1]
                row[2+i] = fmt_value(tid, data[off:off+sz])
        decoded_rows.append(row)
    
    #backfill 
    next_seen = [""] * len(columns)
    for row in reversed(decoded_rows):
        for i in range(len(columns)):
            col = 2 + i
            if row[col] == "":
                row[col] = next_seen[i]
            else:
                next_seen[i] = row[col]

    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp_ms", "sensor"] + [f"{vname}.{leaf[0]}" for _, vname, leaf in columns])
        w.writerows(decoded_rows)

    return len(decoded_rows), len(columns)


def write_eeprom_csv(eeprom_data, eeprom_entries, path):
    root, _ = build_tree(eeprom_entries)
    if root.size > len(eeprom_data):
        print(f"warning: EEPROM struct ({root.size}B) > blob ({len(eeprom_data)}B); truncating\n")

    rows = 0
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["field", "value"])
        for field, tid, off in leaves_of(root):
            sz = LEAVES[tid][1]
            if off + sz > len(eeprom_data):
                break
            w.writerow([field, fmt_value(tid, eeprom_data[off:off+sz])])
            rows += 1
    return rows


def write_meta_csv(metalog_tail, path):
    pos, rows = 0, 0
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["code", "value"])
        while pos + 4 <= len(metalog_tail):
            code = struct.unpack("<I", metalog_tail[pos:pos+4])[0]
            spec = META_CODES.get(code)
            if spec is None:
                print(f"unknown metalog code {code} at offset {pos}; aborting\n")
                break
            name, tid = spec
            sz = LEAVES[tid][1]
            pos += 4
            if pos + sz + 1 > len(metalog_tail):
                print(f"truncated metalog entry for {name}\n")
                break
            value = fmt_value(tid, metalog_tail[pos:pos+sz])
            pos += sz
            if metalog_tail[pos:pos+1] != b"\n":
                print(f"expected \\n after metalog {name}; aborting\n")
                break
            pos += 1
            w.writerow([name, value])
            rows += 1
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("input", type=Path)
    p.add_argument("-o", "--output", type=Path, default=None, help="output prefix (default: input with .launch stripped)")
    args = p.parse_args()

    prefix = args.output or args.input.with_suffix("")
    flight_path = prefix.with_name(prefix.name + "_flight.csv")
    eeprom_path = prefix.with_name(prefix.name + "_eeprom.csv")
    meta_path = prefix.with_name(prefix.name + "_meta.csv")

    hdr, meta_blob, bin_blob = read_launch(args.input)
    print(f"decoding {hdr['name']} v{hdr['version']} (log_csum 0x{hdr['log_csum']:08x}, eep_csum 0x{hdr['eep_csum']:08x}, meta={len(meta_blob)}B, bin={len(bin_blob)}B)")


    log_entries, disc_entries, eeprom_data, eeprom_entries, metalog_tail = read_meta(meta_blob)
    write_eeprom_csv(eeprom_data, eeprom_entries, eeprom_path)
    write_meta_csv(metalog_tail, meta_path)
    write_flight_csv(bin_blob, hdr["log_csum"], log_entries, disc_entries, flight_path)

    print(f"Done.\n")

if __name__ == "__main__":
    main()
