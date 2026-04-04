//! Mode S / ADS-B parity (CRC-24). Matches readsb `crc.c` (`modesChecksum`, generator `0xFFF409`).

const std = @import("std");

const generator_poly: u32 = 0xFFF409;

var crc_table: [256]u32 = undefined;
var syndromes_112: [112]u24 = undefined;
var syndromes_56: [56]u24 = undefined;
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
        syndromes_112[bit] = modesChecksumUnchecked14(&msg);
    }
    for (0..56) |bit| {
        var msg: [7]u8 = [_]u8{0} ** 7;
        msg[bit / 8] ^= @as(u8, 1) << @intCast(7 - (bit % 8));
        syndromes_56[bit] = modesChecksumUnchecked7(&msg);
    }
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

/// 56-bit (7-byte) short message remainder; **0** when parity checks.
pub fn modesChecksum56(message: *const [7]u8) u24 {
    initTablesOnce();
    return modesChecksumUnchecked7(message);
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
    for (0..112) |i| {
        if (syndromes_112[i] == s) {
            msg[i / 8] ^= @as(u8, 1) << @intCast(7 - (i % 8));
            return modesChecksumUnchecked14(msg) == 0;
        }
    }
    return false;
}

/// Short (56-bit) message: valid CRC or single-bit fix to valid.
pub fn acceptOrFixSingleBit56(msg: *[7]u8) bool {
    initTablesOnce();
    if (modesChecksumUnchecked7(msg) == 0) return true;
    const s = modesChecksumUnchecked7(msg);
    for (0..56) |i| {
        if (syndromes_56[i] == s) {
            msg[i / 8] ^= @as(u8, 1) << @intCast(7 - (i % 8));
            return modesChecksumUnchecked7(msg) == 0;
        }
    }
    return false;
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

/// Short (56-bit) message: valid CRC, single-bit fix, or bounded two-bit fix.
pub fn acceptOrFixTwoBit56(msg: *[7]u8, max_pairs: usize) bool {
    initTablesOnce();
    if (acceptOrFixSingleBit56(msg)) return true;

    var tries: usize = 0;
    var i: usize = 0;
    while (i < 56) : (i += 1) {
        var j: usize = i + 1;
        while (j < 56) : (j += 1) {
            msg[i / 8] ^= @as(u8, 1) << @intCast(7 - (i % 8));
            msg[j / 8] ^= @as(u8, 1) << @intCast(7 - (j % 8));
            const ok = modesChecksumUnchecked7(msg) == 0;
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
