//! Decode the 56-bit ADS-B ME field (DF17/18). Not encryption — standard ICAO bit layouts.
//! Bit layouts follow readsb `mode_s.c` (1-based ME bit indices → MSB of `me` is ME bit 1).

const std = @import("std");
const cpr = @import("cpr_decode.zig");

/// ICAO 6-bit character set (AIS / Mode S identification).
pub const ais_charset = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";

pub fn me56FromBytes(msg: *const [14]u8) u56 {
    return (@as(u56, msg[4]) << 48) |
        (@as(u56, msg[5]) << 40) |
        (@as(u56, msg[6]) << 32) |
        (@as(u56, msg[7]) << 24) |
        (@as(u56, msg[8]) << 16) |
        (@as(u56, msg[9]) << 8) |
        @as(u56, msg[10]);
}

/// Reorder 13-bit field into hex Gillham layout (readsb `decodeID13Field`).
fn decodeId13FieldGillham(id13: u32) u32 {
    const x = id13 & 0x1FFF;
    var g: u32 = 0;
    if ((x & 0x1000) != 0) g |= 0x0010;
    if ((x & 0x0800) != 0) g |= 0x1000;
    if ((x & 0x0400) != 0) g |= 0x0020;
    if ((x & 0x0200) != 0) g |= 0x2000;
    if ((x & 0x0100) != 0) g |= 0x0040;
    if ((x & 0x0080) != 0) g |= 0x4000;
    if ((x & 0x0020) != 0) g |= 0x0100;
    if ((x & 0x0010) != 0) g |= 0x0001;
    if ((x & 0x0008) != 0) g |= 0x0200;
    if ((x & 0x0004) != 0) g |= 0x0002;
    if ((x & 0x0002) != 0) g |= 0x0400;
    if ((x & 0x0001) != 0) g |= 0x0004;
    return g;
}

/// Mode A Gillham → Mode C altitude in 100 ft units (dump1090 `ModeAToModeC`).
fn modeAToModeC(mode_a: u32) ?i32 {
    if ((mode_a & 0xFFFF8889) != 0) return null;
    if ((mode_a & 0x000000F0) == 0) return null;

    var one_hundreds: u32 = 0;
    var five_hundreds: u32 = 0;

    if ((mode_a & 0x0010) != 0) one_hundreds ^= 0x007;
    if ((mode_a & 0x0020) != 0) one_hundreds ^= 0x003;
    if ((mode_a & 0x0040) != 0) one_hundreds ^= 0x001;

    if ((one_hundreds & 5) == 5) one_hundreds ^= 2;

    if (one_hundreds > 5) return null;

    if ((mode_a & 0x0002) != 0) five_hundreds ^= 0x0FF;
    if ((mode_a & 0x0004) != 0) five_hundreds ^= 0x07F;

    if ((mode_a & 0x1000) != 0) five_hundreds ^= 0x03F;
    if ((mode_a & 0x2000) != 0) five_hundreds ^= 0x01F;
    if ((mode_a & 0x4000) != 0) five_hundreds ^= 0x00F;

    if ((mode_a & 0x0100) != 0) five_hundreds ^= 0x007;
    if ((mode_a & 0x0200) != 0) five_hundreds ^= 0x003;
    if ((mode_a & 0x0400) != 0) five_hundreds ^= 0x001;

    if ((five_hundreds & 1) != 0) {
        one_hundreds = 6 - one_hundreds;
    }

    const n = (@as(i32, @intCast(five_hundreds)) * 5) + @as(i32, @intCast(one_hundreds)) - 13;
    if (n < -12) return null;
    return n;
}

/// 12-bit AC altitude (feet): Q=1 → 25 ft steps; Q=0 → Gillham Mode C (100 ft); else null.
pub fn decodeAc12Feet(ac12: u12) ?i32 {
    const a = @as(u32, ac12);
    if ((a & 0x10) != 0) {
        const n = ((a & 0x0FE0) >> 1) | (a & 0x000F);
        return @as(i32, @intCast(n)) * 25 - 1000;
    }
    const n13 = ((a & 0x0FC0) << 1) | (a & 0x003F);
    const mode_a = decodeId13FieldGillham(n13);
    const n = modeAToModeC(mode_a) orelse return null;
    return 100 * n;
}

/// 13-bit AC (DF 4 / DF 20): Q=1 → 25 ft; Q=0 → Gillham Mode C; M=1 → meters (unsupported).
pub fn decodeAc13Feet(ac13: u32) ?i32 {
    const m_bit = ac13 & 0x0040;
    const q_bit = ac13 & 0x0010;
    if (m_bit != 0) return null;
    if (q_bit != 0) {
        const n = ((ac13 & 0x1F80) >> 2) | ((ac13 & 0x0020) >> 1) | (ac13 & 0x000F);
        return @as(i32, @intCast(n)) * 25 - 1000;
    }
    const n = modeAToModeC(decodeId13FieldGillham(ac13)) orelse return null;
    if (n < -12) return null;
    return 100 * n;
}

pub fn decodeCallsign(me: u56, buf: *[9]u8) void {
    var i: u32 = 0;
    while (i < 8) : (i += 1) {
        const shift: u6 = @intCast(42 - 6 * @as(i32, @intCast(i)));
        const idx: u8 = @truncate((me >> shift) & 0x3F);
        buf.*[i] = ais_charset[idx];
    }
    buf.*[8] = 0;
}

pub fn trimCallsignPadding(buf: *[9]u8) []const u8 {
    var end: usize = 8;
    while (end > 0 and buf.*[end - 1] == ' ') end -= 1;
    return buf.*[0..end];
}

pub const AirbornePosition = struct {
    ss: u2,
    nic_b: u1,
    alt12: u12,
    t: bool,
    f_odd: bool,
    lat_cpr: u32,
    lon_cpr: u32,
};

/// TC (metype) 9–18 airborne position; `tc == 0` means no CPR in message.
pub fn parseAirbornePosition(me: u56, tc: u32) ?AirbornePosition {
    if (tc == 0) return null;
    if (tc < 9 or tc > 18) return null;

    return .{
        .ss = @truncate((me >> 49) & 0x3),
        .nic_b = @truncate((me >> 48) & 1),
        .alt12 = @truncate((me >> 36) & 0xFFF),
        .t = ((me >> 35) & 1) != 0,
        .f_odd = ((me >> 34) & 1) != 0,
        .lat_cpr = @truncate((me >> 17) & 0x1FFFF),
        .lon_cpr = @truncate(me & 0x1FFFF),
    };
}

pub const SurfacePosition = struct {
    movement: u7,
    heading_valid: bool,
    heading_deg: f32,
    t_flag: bool,
    f_odd: bool,
    lat_cpr: u32,
    lon_cpr: u32,
};

/// TC 5–8 surface position (readsb `decodeESSurfacePosition` ME layout).
pub fn parseSurfacePosition(me: u56, tc: u32) ?SurfacePosition {
    if (tc < 5 or tc > 8) return null;

    const movement: u7 = @truncate((me >> 44) & 0x7F);
    const hdg_stat = ((me >> 43) & 1) != 0;
    const hdg_raw: u32 = @truncate((me >> 36) & 0x7F);
    const heading_deg = if (hdg_stat) @as(f32, @floatFromInt(hdg_raw)) * (360.0 / 128.0) else @as(f32, 0);

    return .{
        .movement = movement,
        .heading_valid = hdg_stat,
        .heading_deg = heading_deg,
        .t_flag = ((me >> 35) & 1) != 0,
        .f_odd = ((me >> 34) & 1) != 0,
        .lat_cpr = @truncate((me >> 17) & 0x1FFFF),
        .lon_cpr = @truncate(me & 0x1FFFF),
    };
}

/// ADS-B movement field v0 (ground speed kt, midpoint; readsb `decodeMovementFieldV0`).
pub fn decodeMovementFieldV0(movement: u32) f32 {
    if (movement == 0 or movement >= 125) return 0;
    if (movement == 124) return 180;
    if (movement >= 109) return 100.0 + (@as(f32, @floatFromInt(movement)) - 109.0 + 0.5) * 5.0;
    if (movement >= 94) return 70.0 + (@as(f32, @floatFromInt(movement)) - 94.0 + 0.5) * 2.0;
    if (movement >= 39) return 15.0 + (@as(f32, @floatFromInt(movement)) - 39.0 + 0.5) * 1.0;
    if (movement >= 13) return 2.0 + (@as(f32, @floatFromInt(movement)) - 13.0 + 0.5) * 0.50;
    if (movement >= 9) return 1.0 + (@as(f32, @floatFromInt(movement)) - 9.0 + 0.5) * 0.25;
    if (movement >= 2) return 0.125 + (@as(f32, @floatFromInt(movement)) - 2.0 + 0.5) * 0.125;
    return 0;
}

pub const VelocityEwNs = struct {
    st: u3,
    nac_v: u3,
    ground_speed_kt: f32,
    track_deg: f32,
    vert_rate_fpm: ?i32,
    /// Meaningful when `vert_rate_fpm` is non-null (ME bit 36).
    vert_rate_is_baro: bool,
    geom_delta_ft: ?i32,
};

/// TC 19 airborne velocity; ST 1–2 (ground speed + track) fully decoded; ST 3–4 partial.
pub fn parseVelocity(me: u56, tc: u32) ?VelocityEwNs {
    if (tc != 19) return null;
    const st: u3 = @truncate((me >> 48) & 0x7);
    if (st < 1 or st > 4) return null;

    const nac_v: u3 = @truncate((me >> 45) & 0x7);

    var gs: f32 = 0;
    var trk: f32 = 0;
    var vr: ?i32 = null;
    var vr_baro: bool = false;
    var gd: ?i32 = null;

    if (st == 1 or st == 2) {
        const ew_raw = @as(u32, @truncate((me >> 32) & 0x3FF));
        const ns_raw = @as(u32, @truncate((me >> 21) & 0x3FF));
        if (ew_raw == 0 or ns_raw == 0) return null;

        const mult: f32 = if (st == 2) 4.0 else 1.0;
        const ew_dir = (me >> 42) & 1;
        const ns_dir = (me >> 31) & 1;
        const ew_vel = @as(f32, @floatFromInt(@as(i32, @intCast(ew_raw - 1)))) * (if (ew_dir != 0) -mult else mult);
        const ns_vel = @as(f32, @floatFromInt(@as(i32, @intCast(ns_raw - 1)))) * (if (ns_dir != 0) -mult else mult);
        gs = @sqrt(ns_vel * ns_vel + ew_vel * ew_vel + 0.5);
        if (gs > 0) {
            var gt = std.math.atan2(ew_vel, ns_vel) * 180.0 / std.math.pi;
            if (gt < 0) gt += 360.0;
            trk = gt;
        }
    } else {
        // ST 3/4: magnetic/true heading + airspeed; ground speed not in same form
        if (((me >> 42) & 1) != 0) {
            const hdg = @as(f32, @floatFromInt((me >> 32) & 0x3FF)) * 360.0 / 1024.0;
            trk = hdg;
        }
        const air_raw = @as(u32, @truncate((me >> 21) & 0x3FF));
        if (air_raw != 0) {
            const mult: f32 = if (st == 4) 4.0 else 1.0;
            gs = @as(f32, @floatFromInt(@as(i32, @intCast(air_raw - 1)))) * mult;
        }
    }

    const vert_rate_raw = @as(u32, @truncate((me >> 10) & 0x1FF));
    if (vert_rate_raw != 0) {
        vr_baro = ((me >> 20) & 1) != 0;
        const sign = ((me >> 19) & 1) != 0;
        const mag = @as(i32, @intCast(vert_rate_raw - 1)) * 64;
        vr = if (sign) -mag else mag;
    }

    const raw_delta = @as(u32, @truncate(me & 0x7F));
    if (raw_delta != 0) {
        const neg = ((me >> 7) & 1) != 0;
        const d = @as(i32, @intCast(raw_delta - 1)) * 25;
        gd = if (neg) -d else d;
    }

    return .{
        .st = st,
        .nac_v = nac_v,
        .ground_speed_kt = gs,
        .track_deg = trk,
        .vert_rate_fpm = vr,
        .vert_rate_is_baro = vr_baro,
        .geom_delta_ft = gd,
    };
}

pub const CprAirborneState = struct {
    even_lat: ?u32 = null,
    even_lon: ?u32 = null,
    even_seen_ns: ?i128 = null,
    odd_lat: ?u32 = null,
    odd_lon: ?u32 = null,
    odd_seen_ns: ?i128 = null,
    last_lat: ?f64 = null,
    last_lon: ?f64 = null,
};

pub const CprSurfaceState = struct {
    even_lat: ?u32 = null,
    even_lon: ?u32 = null,
    even_seen_ns: ?i128 = null,
    odd_lat: ?u32 = null,
    odd_lon: ?u32 = null,
    odd_seen_ns: ?i128 = null,
    last_lat: ?f64 = null,
    last_lon: ?f64 = null,
};

pub fn updateCprAndMaybeDecode(
    state: *CprAirborneState,
    pos: AirbornePosition,
    fflag_odd: bool,
    now_ns: i128,
    receiver_lat: ?f64,
    receiver_lon: ?f64,
) ?struct { lat: f64, lon: f64, global: bool } {
    // The CPR position is only valid if the even/odd frames are temporally close.
    // If we combine stale pairs, the decoded position can be wildly wrong.
    const CPR_PAIR_MAX_AGE_NS: i128 = 10 * std.time.ns_per_s;

    if (pos.f_odd) {
        state.odd_lat = pos.lat_cpr;
        state.odd_lon = pos.lon_cpr;
        state.odd_seen_ns = now_ns;
    } else {
        state.even_lat = pos.lat_cpr;
        state.even_lon = pos.lon_cpr;
        state.even_seen_ns = now_ns;
    }

    if (state.even_lat != null and state.even_lon != null and
        state.odd_lat != null and state.odd_lon != null)
    {
        if (state.even_seen_ns != null and state.odd_seen_ns != null) {
            const even_age = now_ns - state.even_seen_ns.?;
            const odd_age = now_ns - state.odd_seen_ns.?;
            if (even_age <= CPR_PAIR_MAX_AGE_NS and odd_age <= CPR_PAIR_MAX_AGE_NS) {
                const dec = cpr.decodeCprAirborne(
                    state.even_lat.?,
                    state.even_lon.?,
                    state.odd_lat.?,
                    state.odd_lon.?,
                    fflag_odd,
                );
                if (dec) |p| {
                    state.last_lat = p.lat;
                    state.last_lon = p.lon;
                    return .{ .lat = p.lat, .lon = p.lon, .global = true };
                }
            }
        }
    }

    const rlat = state.last_lat orelse receiver_lat;
    const rlon = state.last_lon orelse receiver_lon;
    if (rlat != null and rlon != null) {
        if (cpr.decodeCprAirborneRelative(
            rlat.?,
            rlon.?,
            pos.lat_cpr,
            pos.lon_cpr,
            pos.f_odd,
        )) |p| {
            state.last_lat = p.lat;
            state.last_lon = p.lon;
            return .{ .lat = p.lat, .lon = p.lon, .global = false };
        }
    }
    return null;
}

pub fn updateCprSurfaceAndMaybeDecode(
    state: *CprSurfaceState,
    pos: SurfacePosition,
    fflag_odd: bool,
    now_ns: i128,
    receiver_lat: ?f64,
    receiver_lon: ?f64,
) ?struct { lat: f64, lon: f64, global: bool } {
    const CPR_PAIR_MAX_AGE_NS: i128 = 10 * std.time.ns_per_s;

    if (pos.f_odd) {
        state.odd_lat = pos.lat_cpr;
        state.odd_lon = pos.lon_cpr;
        state.odd_seen_ns = now_ns;
    } else {
        state.even_lat = pos.lat_cpr;
        state.even_lon = pos.lon_cpr;
        state.even_seen_ns = now_ns;
    }

    const ref_lat = state.last_lat orelse receiver_lat orelse 0.0;
    const ref_lon = state.last_lon orelse receiver_lon orelse 0.0;

    if (state.even_lat != null and state.even_lon != null and
        state.odd_lat != null and state.odd_lon != null)
    {
        if (state.even_seen_ns != null and state.odd_seen_ns != null) {
            const even_age = now_ns - state.even_seen_ns.?;
            const odd_age = now_ns - state.odd_seen_ns.?;
            if (even_age <= CPR_PAIR_MAX_AGE_NS and odd_age <= CPR_PAIR_MAX_AGE_NS) {
                if (cpr.decodeCprSurface(
                    ref_lat,
                    ref_lon,
                    state.even_lat.?,
                    state.even_lon.?,
                    state.odd_lat.?,
                    state.odd_lon.?,
                    fflag_odd,
                )) |p| {
                    state.last_lat = p.lat;
                    state.last_lon = p.lon;
                    return .{ .lat = p.lat, .lon = p.lon, .global = true };
                }
            }
        }
    }

    const srlat = state.last_lat orelse receiver_lat;
    const srlon = state.last_lon orelse receiver_lon;
    if (srlat != null and srlon != null) {
        if (cpr.decodeCprSurfaceRelative(
            srlat.?,
            srlon.?,
            pos.lat_cpr,
            pos.lon_cpr,
            pos.f_odd,
        )) |p| {
            state.last_lat = p.lat;
            state.last_lon = p.lon;
            return .{ .lat = p.lat, .lon = p.lon, .global = false };
        }
    }
    return null;
}

/// Log one message to stderr (for `--verbose`). Does **not** CPR state; CPR is applied in `aircraft_table.updateFromEsMessage` (with caller-supplied `now_ns`).
pub fn printPayloadDecodes(
    msg: *const [14]u8,
    df: u32,
    ca: u32,
    icao: u32,
    tc: u32,
    conf: f64,
) void {
    const me = me56FromBytes(msg);

    std.debug.print("DF={d} CA={d} ICAO={x:0>6} TC={d} conf={d:.5} RAW=", .{ df, ca, icao, tc, conf });
    for (msg) |b| std.debug.print("{x:0>2}", .{b});
    std.debug.print("\n", .{});

    if (tc >= 1 and tc <= 4) {
        var callsign_buf: [9]u8 = undefined;
        decodeCallsign(me, &callsign_buf);
        const cs = trimCallsignPadding(&callsign_buf);
        std.debug.print("  ident: {s}\n", .{cs});
    }

    if (parseSurfacePosition(me, tc)) |sp| {
        if (sp.movement > 0 and sp.movement < 125) {
            const gs = decodeMovementFieldV0(sp.movement);
            std.debug.print("  gnd: speed≈{d:.1} kt (movement {d})\n", .{ gs, sp.movement });
        }
        if (sp.heading_valid) {
            std.debug.print("  gnd: hdg={d:.1}°\n", .{sp.heading_deg});
        }
        std.debug.print("  CPR(surf): {s} lat={d} lon={d} (T={})\n", .{
            if (sp.f_odd) "odd" else "even",
            sp.lat_cpr,
            sp.lon_cpr,
            sp.t_flag,
        });
    }

    if (parseAirbornePosition(me, tc)) |pos| {
        if (decodeAc12Feet(pos.alt12)) |alt_ft| {
            const q = (pos.alt12 & 0x10) != 0;
            std.debug.print("  alt: {d} ft (baro, {s})\n", .{ alt_ft, if (q) "Q" else "Gillham" });
        } else {
            std.debug.print("  alt: 0x{x:0>3} (12-bit, invalid or meters)\n", .{pos.alt12});
        }
        std.debug.print("  CPR: {s} frame lat={d} lon={d} (T={}) ss={d} nic_b={d}\n", .{
            if (pos.f_odd) "odd" else "even",
            pos.lat_cpr,
            pos.lon_cpr,
            pos.t,
            pos.ss,
            pos.nic_b,
        });
    }

    if (parseVelocity(me, tc)) |v| {
        std.debug.print("  vel: ST={d} NACv={d} gs={d:.1} kt trk={d:.1}°", .{
            v.st,
            v.nac_v,
            v.ground_speed_kt,
            v.track_deg,
        });
        if (v.vert_rate_fpm) |r| {
            std.debug.print(" vrate={d} ft/min ({s})", .{
                r,
                if (v.vert_rate_is_baro) "baro" else "geom",
            });
        }
        if (v.geom_delta_ft) |d| {
            std.debug.print(" geom-baro={d} ft", .{d});
        }
        std.debug.print("\n", .{});
    }
}
