//! Beast binary output matching flightaware/dump1090 `writeBeastMessage` (cooked path).

const std = @import("std");

fn appendEscaped(buf: []u8, out_len: *usize, byte: u8) error{NoSpace}!void {
    if (out_len.* + 1 + @as(usize, @intFromBool(byte == 0x1a)) > buf.len) return error.NoSpace;
    buf[out_len.*] = byte;
    out_len.* += 1;
    if (byte == 0x1a) {
        buf[out_len.*] = 0x1a;
        out_len.* += 1;
    }
}

/// 48-bit 12 MHz receive timestamp (lower 48 bits of tick counter), big-endian in wire format.
pub fn timestamp12MHz(now_ns: i128) u48 {
    const ticks: u128 = @as(u128, @intCast(now_ns)) * 12_000_000 / std.time.ns_per_s;
    return @truncate(@as(u48, @intCast(ticks & 0xFFFFFFFFFFFF)));
}

/// dump1090: `sig = round(sqrt(signalLevel) * 255)` with clamps; `signalLevel` is demod power.
pub fn signalLevelToRssiByte(signal_level: f64) u8 {
    var sig: i32 = @intFromFloat(@round(@sqrt(signal_level) * 255.0));
    if (signal_level > 0 and sig < 1) sig = 1;
    if (sig > 255) sig = 255;
    if (sig < 0) sig = 0;
    return @intCast(sig);
}

/// Encodes one Beast frame (cooked Mode S). `msg` is 7 or 14 bytes. Returns wire length.
pub fn encodeModeSFrame(buf: []u8, timestamp48: u48, signal_level: f64, msg: []const u8) error{ NoSpace, BadModeSLength }!usize {
    const msg_len = msg.len;
    const type_ch: u8 = switch (msg_len) {
        7 => '2',
        14 => '3',
        2 => '1',
        else => return error.BadModeSLength,
    };

    var len: usize = 0;
    try appendEscaped(buf, &len, 0x1a);
    try appendEscaped(buf, &len, type_ch);

    inline for (0..6) |k| {
        const shift: u6 = @intCast(40 - 8 * k);
        const byte = @as(u8, @truncate((@as(u48, timestamp48) >> shift) & 0xff));
        try appendEscaped(buf, &len, byte);
    }

    const rssi = signalLevelToRssiByte(signal_level);
    try appendEscaped(buf, &len, rssi);

    for (msg) |b| {
        try appendEscaped(buf, &len, b);
    }
    return len;
}

/// Beast heartbeat frame from dump1090 `send_beast_heartbeat`.
pub fn encodeHeartbeat(buf: []u8) error{NoSpace}!usize {
    const heartbeat = [_]u8{ 0x1a, '1', 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    if (heartbeat.len > buf.len) return error.NoSpace;
    @memcpy(buf[0..heartbeat.len], &heartbeat);
    return heartbeat.len;
}
