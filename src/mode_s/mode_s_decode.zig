//! Downlink format routing (dump1090-style: `(df & 0x10) != 0` → 112 bits else 56).

const std = @import("std");
const adsb = @import("../adsb/adsb_payload.zig");

/// ICAO / dump1090: DF with bit 0x10 set uses long (112-bit) messages.
pub fn modeSBitLength(df: u32) usize {
    return if ((df & 0x10) != 0) 112 else 56;
}

/// Bit indices are **1-based** (Annex 10 style), inclusive.
pub fn bitsInclusive1(msg: []const u8, first_bit: u32, last_bit: u32) u32 {
    const len = last_bit - first_bit + 1;
    var acc: u32 = 0;
    var i: u32 = 0;
    while (i < len) : (i += 1) {
        const bi = first_bit - 1 + i;
        const b = (msg[bi / 8] >> @intCast(7 - (bi % 8))) & 1;
        acc = (acc << 1) | b;
    }
    return acc;
}

pub fn printVerbose(
    msg: []const u8,
    bit_len: usize,
    df: u32,
    conf: f64,
) void {
    switch (df) {
        17, 18 => {
            if (bit_len == 112) {
                const ca = msg[0] & 0x07;
                const icao: u32 = (@as(u32, msg[1]) << 16) | (@as(u32, msg[2]) << 8) | @as(u32, msg[3]);
                const tc = (msg[4] >> 3) & 0x1f;
                var msg14: [14]u8 = undefined;
                @memcpy(&msg14, msg[0..14]);
                adsb.printPayloadDecodes(&msg14, df, ca, icao, tc, conf);
            }
            return;
        },
        else => {},
    }

    std.debug.print("DF={d} conf={d:.5} bits={d} RAW=", .{ df, conf, bit_len });
    if (bit_len == 56) {
        for (msg[0..7]) |b| std.debug.print("{x:0>2}", .{b});
    } else {
        for (msg[0..14]) |b| std.debug.print("{x:0>2}", .{b});
    }
    std.debug.print("\n", .{});

    switch (df) {
        0 => {
            const vs = bitsInclusive1(msg, 6, 6) != 0;
            std.debug.print("  DF0 air-air: VS={} RI={d} SL={d}\n", .{
                vs,
                bitsInclusive1(msg, 14, 17),
                bitsInclusive1(msg, 9, 11),
            });
        },
        4 => {
            const ac = bitsInclusive1(msg, 20, 32);
            if (adsb.decodeAc13Feet(ac)) |alt| {
                std.debug.print("  DF4 surveillance alt: {d} ft (baro)\n", .{alt});
            }
        },
        5 => {
            std.debug.print("  DF5 surveillance ID Gillham 0x{x:0>4}\n", .{bitsInclusive1(msg, 20, 32) & 0x1FFF});
        },
        11 => {
            const aa = bitsInclusive1(msg, 9, 32);
            std.debug.print("  DF11 all-call AA={x:0>6} CA={d}\n", .{ aa, bitsInclusive1(msg, 6, 8) });
        },
        16 => std.debug.print("  DF16 long air-air (MV)\n", .{}),
        20 => {
            const ac = bitsInclusive1(msg, 20, 32);
            if (adsb.decodeAc13Feet(ac)) |alt| {
                std.debug.print("  DF20 Comm-B alt: {d} ft\n", .{alt});
            }
        },
        21 => std.debug.print("  DF21 Comm-B identity / squawk path\n", .{}),
        24...31 => std.debug.print("  DF{d} Comm-D ELM segment\n", .{df}),
        else => {},
    }
}
