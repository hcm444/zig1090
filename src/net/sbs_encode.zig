//! BaseStation / SBS TCP output matching flightaware/dump1090 `modesSendSBSOutput` (baro path, no `--gnss`).

const std = @import("std");
const builtin = @import("builtin");
const c = @cImport({
    @cInclude("time.h");
});
const adsb = @import("../adsb/adsb_payload.zig");
const msdec = @import("../mode_s/mode_s_decode.zig");
const aircraft_table = @import("../ui/aircraft_table.zig");

fn formatWallClock(ymd: *[10]u8, hms: *[12]u8, ms_since_epoch: i128) void {
    const sec = @divFloor(ms_since_epoch, 1000);
    const milli = @as(u32, @intCast(@mod(ms_since_epoch, 1000)));
    var t: c.time_t = @intCast(sec);
    var tm: c.struct_tm = undefined;
    const ok = if (builtin.os.tag == .windows)
        c.localtime_s(&tm, &t) == 0
    else
        c.localtime_r(&t, &tm) != null;
    if (!ok) {
        @memcpy(ymd, "1970/01/01");
        @memcpy(hms, "00:00:00.000");
        return;
    }
    _ = std.fmt.bufPrint(ymd, "{d:0>4}/{d:0>2}/{d:0>2}", .{ @as(i32, tm.tm_year) + 1900, tm.tm_mon + 1, tm.tm_mday }) catch {};
    _ = std.fmt.bufPrint(hms, "{d:0>2}:{d:0>2}:{d:0>2}.{d:0>3}", .{ tm.tm_hour, tm.tm_min, tm.tm_sec, milli }) catch {};
}

fn callsignSlice(ac: *const aircraft_table.Aircraft) []const u8 {
    var end: usize = ac.callsign_len;
    while (end > 0 and ac.callsign[end - 1] == ' ') end -= 1;
    return ac.callsign[0..end];
}

/// Returns `null` when dump1090 would skip SBS for this message.
pub fn formatSbsLine(
    buf: []u8,
    msg: []const u8,
    bit_len: usize,
    df: u32,
    now_ns: i128,
    crc_repair_bits: u8,
    ac: *const aircraft_table.Aircraft,
) ?[]const u8 {
    if (crc_repair_bits >= 2) return null;

    const recv_ms = @divFloor(now_ns, std.time.ns_per_ms);
    const now_ms = recv_ms;

    var ymd_recv: [10]u8 = undefined;
    var hms_recv: [12]u8 = undefined;
    var ymd_now: [10]u8 = undefined;
    var hms_now: [12]u8 = undefined;
    formatWallClock(&ymd_recv, &hms_recv, recv_ms);
    formatWallClock(&ymd_now, &hms_now, now_ms);

    const msg_type: i32 = switch (df) {
        4, 20 => 5,
        5, 21 => 6,
        0, 16 => 7,
        11 => 8,
        17, 18 => blk: {
            if (bit_len != 112) return null;
            const tc = (msg[4] >> 3) & 0x1f;
            if (tc >= 1 and tc <= 4) break :blk 1;
            if (tc >= 5 and tc <= 8) break :blk 2;
            if (tc >= 9 and tc <= 18) break :blk 3;
            if (tc == 19) break :blk 4;
            return null;
        },
        else => return null,
    };

    var cs_buf: [9]u8 = undefined;
    const flight: []const u8 = blk: {
        if (df == 17 or df == 18) {
            const tc = (msg[4] >> 3) & 0x1f;
            if (tc >= 1 and tc <= 4) {
                var m14: [14]u8 = undefined;
                @memcpy(&m14, msg[0..14]);
                const me = adsb.me56FromBytes(&m14);
                adsb.decodeCallsign(me, &cs_buf);
                break :blk adsb.trimCallsignPadding(&cs_buf);
            }
        }
        break :blk callsignSlice(ac);
    };

    var alt_field: [32]u8 = undefined;
    const alt_field_s: []const u8 = blk: {
        if (df == 4 or df == 20) {
            const ac13 = msdec.bitsInclusive1(msg, 20, 32);
            if (adsb.decodeAc13Feet(ac13)) |feet| {
                break :blk std.fmt.bufPrint(&alt_field, "{d}", .{feet}) catch "";
            }
        }
        if (ac.baro_alt_ft) |a| {
            break :blk std.fmt.bufPrint(&alt_field, "{d}", .{a}) catch "";
        }
        if (ac.geom_alt_ft) |g| {
            break :blk std.fmt.bufPrint(&alt_field, "{d}", .{g}) catch "";
        }
        break :blk "";
    };

    var gs_str: [16]u8 = undefined;
    const gs_s: []const u8 = if (ac.ground_speed_kt) |gs| std.fmt.bufPrint(&gs_str, "{d:.0}", .{gs}) catch "" else "";

    var trk_str: [16]u8 = undefined;
    const trk_s: []const u8 = if (ac.track_deg) |tr| std.fmt.bufPrint(&trk_str, "{d:.0}", .{tr}) catch "" else "";

    var lat_buf: [24]u8 = undefined;
    var lon_buf: [24]u8 = undefined;
    const lat_s: []const u8 = if (ac.lat) |la| std.fmt.bufPrint(&lat_buf, "{d:.5}", .{la}) catch "" else "";
    const lon_s: []const u8 = if (ac.lon) |lo| std.fmt.bufPrint(&lon_buf, "{d:.5}", .{lo}) catch "" else "";

    var vr_str: [16]u8 = undefined;
    const vr_s: []const u8 = if (ac.vert_rate_fpm) |vr| std.fmt.bufPrint(&vr_str, "{d}", .{vr}) catch "" else "";

    var sq_str: [8]u8 = undefined;
    const sq_s: []const u8 = if (ac.squawk) |sq| std.fmt.bufPrint(&sq_str, "{x:0>4}", .{@as(u32, sq)}) catch "" else "";

    const alert_v: []const u8 = ""; // alert flag not tracked separately
    const emerg_v: []const u8 = if (ac.squawk) |sq| if (sq == 7500 or sq == 7600 or sq == 7700) "-1" else "0" else "";
    const spi_v: []const u8 = "0";
    const og_v: []const u8 = if (ac.on_ground) "-1" else if (ac.lat != null) "0" else "";

    const line = std.fmt.bufPrint(
        buf,
        "MSG,{d},1,1,{x:0>6},1,{s},{s},{s},{s},{s},{s},{s},{s},{s},{s},{s},{s},{s},{s},{s},{s}\r\n",
        .{
            msg_type,
            ac.icao,
            &ymd_recv,
            &hms_recv,
            &ymd_now,
            &hms_now,
            flight,
            alt_field_s,
            gs_s,
            trk_s,
            lat_s,
            lon_s,
            vr_s,
            sq_s,
            alert_v,
            emerg_v,
            spi_v,
            og_v,
        },
    ) catch return null;

    return line;
}
