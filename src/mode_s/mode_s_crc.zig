//! Mode S / ADS-B parity (CRC-24). Matches readsb `crc.c` (`modesChecksum`, generator `0xFFF409`).

const std = @import("std");

const generator_poly: u32 = 0xFFF409;

var crc_table: [256]u32 = undefined;

const Syndrome112Pair = struct { syndrome: u24, bit: u8 };
/// Single-bit error syndromes sorted by `(syndrome, bit_index)` for binary search (same semantics as linear scan 0..111).
var syndromes_112_sorted: [112]Syndrome112Pair = undefined;
var tables_init: bool = false;

fn initTablesOnce() void {
    if (tables_init) return;
    tables_init = true;

    for (0..256) |i| {
        var c: u32 = @as(u32, @intCast(i)) << 16;
        for (0..8) |_| {
            if (c & 0x800000 != 0) {
                c = (c << 1) ^ generator_poly;
            } else {
                c = c << 1;
            }
        }
        crc_table[i] = c & 0x00ffffff;
    }

    for (0..112) |bit| {
        var msg: [14]u8 = [_]u8{0} ** 14;
        msg[bit / 8] ^= @as(u8, 1) << @intCast(7 - (bit % 8));
        syndromes_112_sorted[bit] = .{
            .syndrome = modesChecksumUnchecked14(&msg),
            .bit = @intCast(bit),
        };
    }
    std.sort.pdq(Syndrome112Pair, &syndromes_112_sorted, {}, struct {
        fn less(_: void, a: Syndrome112Pair, b: Syndrome112Pair) bool {
            if (a.syndrome != b.syndrome) return a.syndrome < b.syndrome;
            return a.bit < b.bit;
        }
    }.less);
}

fn syndrome112LowerBound(s: u24) usize {
    var lo: usize = 0;
    var hi: usize = syndromes_112_sorted.len;
    while (lo < hi) {
        const mid = lo + (hi - lo) / 2;
        if (syndromes_112_sorted[mid].syndrome < s) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

/// Internal: assumes `crc_table` is initialized.
fn modesChecksumUnchecked14(message: *const [14]u8) u24 {
    const n: usize = 14;
    var rem: u32 = 0;
    var i: usize = 0;
    while (i < n - 3) : (i += 1) {
        const top: u8 = @truncate((rem & 0xff0000) >> 16);
        rem = ((rem << 8) ^ crc_table[message[i] ^ top]) & 0xffffff;
    }
    rem ^= @as(u32, message[n - 3]) << 16;
    rem ^= @as(u32, message[n - 2]) << 8;
    rem ^= @as(u32, message[n - 1]);
    return @truncate(rem & 0xffffff);
}

fn modesChecksumUnchecked7(message: *const [7]u8) u24 {
    const n: usize = 7;
    var rem: u32 = 0;
    var i: usize = 0;
    while (i < n - 3) : (i += 1) {
        const top: u8 = @truncate((rem & 0xff0000) >> 16);
        rem = ((rem << 8) ^ crc_table[message[i] ^ top]) & 0xffffff;
    }
    rem ^= @as(u32, message[n - 3]) << 16;
    rem ^= @as(u32, message[n - 2]) << 8;
    rem ^= @as(u32, message[n - 1]);
    return @truncate(rem & 0xffffff);
}

/// 112-bit (14-byte) Mode S remainder; **0** means parity checks (DF17/18 ES).
pub fn modesChecksum(message: *const [14]u8) u24 {
    initTablesOnce();
    return modesChecksumUnchecked14(message);
}

/// 56-bit (7-byte) CRC remainder (same polynomial as readsb `modesChecksum`).
/// Meaning depends on DF: **DF11** (PI) upper 17 bits are zero when valid (II may occupy low 7);
/// **DF 0/4/5** (address parity) equals the 24-bit ICAO address; **not** generally zero.
pub fn modesChecksum56(message: *const [7]u8) u24 {
    initTablesOnce();
    return modesChecksumUnchecked7(message);
}

/// DF11 all-call (parity/interrogator): valid when only the low 7 bits may be nonzero (dump1090 `correctMessage`).
pub fn df11ParityOk(s: u24) bool {
    return (s & 0xFFFF80) == 0;
}

/// Accept a 56-bit frame per downlink parity type (dump1090 `decodeModesMessage` / `correctMessage`).
/// - **DF 0, 4, 5**: address parity — remainder is the aircraft address; always accept (no bit-fix to zero).
/// - **DF 11**: parity/interrogator — accept if `(remainder & 0xFFFF80) == 0`, or try a single-bit flip to achieve that.
/// - **Other** short DFs: reject (we do not validate their parity here).
pub fn accept56Short(df: u32, msg: *[7]u8, allow_single_bit_fix_df11: bool) bool {
    initTablesOnce();
    if (df == 0 or df == 4 or df == 5) return true;
    if (df != 11) return false;

    if (df11ParityOk(modesChecksumUnchecked7(msg))) return true;
    if (!allow_single_bit_fix_df11) return false;

    for (0..56) |i| {
        msg[i / 8] ^= @as(u8, 1) << @intCast(7 - (i % 8));
        if (df11ParityOk(modesChecksumUnchecked7(msg))) return true;
        msg[i / 8] ^= @as(u8, 1) << @intCast(7 - (i % 8));
    }
    return false;
}

/// Call once from `main` before hot loop (optional; lazy init also works).
pub fn initTables() void {
    initTablesOnce();
}

/// Returns true if CRC is already valid, or fixes **one** bit in place and returns true.
pub fn acceptOrFixSingleBit(msg: *[14]u8) bool {
    initTablesOnce();
    if (modesChecksumUnchecked14(msg) == 0) return true;
    const s = modesChecksumUnchecked14(msg);
    const idx = syndrome112LowerBound(s);
    if (idx >= syndromes_112_sorted.len or syndromes_112_sorted[idx].syndrome != s) return false;
    const bit = syndromes_112_sorted[idx].bit;
    msg[bit / 8] ^= @as(u8, 1) << @intCast(7 - (bit % 8));
    return modesChecksumUnchecked14(msg) == 0;
}

/// Long (112-bit) message: valid CRC, single-bit fix, or bounded two-bit fix.
pub fn acceptOrFixTwoBit(msg: *[14]u8, max_pairs: usize) bool {
    initTablesOnce();
    if (acceptOrFixSingleBit(msg)) return true;

    var tries: usize = 0;
    var i: usize = 0;
    while (i < 112) : (i += 1) {
        var j: usize = i + 1;
        while (j < 112) : (j += 1) {
            msg[i / 8] ^= @as(u8, 1) << @intCast(7 - (i % 8));
            msg[j / 8] ^= @as(u8, 1) << @intCast(7 - (j % 8));
            const ok = modesChecksumUnchecked14(msg) == 0;
            if (ok) return true;
            msg[i / 8] ^= @as(u8, 1) << @intCast(7 - (i % 8));
            msg[j / 8] ^= @as(u8, 1) << @intCast(7 - (j % 8));

            tries += 1;
            if (tries >= max_pairs) return false;
        }
    }
    return false;
}

test "crc init" {
    initTables();
    try std.testing.expect(crc_table[1] != 0);
}

test "df11 parity mask matches dump1090" {
    try std.testing.expect(df11ParityOk(0));
    try std.testing.expect(df11ParityOk(0x7f)); // II only
    try std.testing.expect(!df11ParityOk(0x80));
    try std.testing.expect(!df11ParityOk(0xFFFFFF));
}
