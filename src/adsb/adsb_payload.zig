//! Decode the 56-bit ADS-B ME field (DF17/18). Not encryption - standard ICAO bit layouts.
//! This module follows Annex 10 / DO-260 bit indexing (ME bit 1 is the MSB of `me`).

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

/// ADS-B Aircraft Identification (TC 1–4) emitter category (EC, 3 bits).
/// Layout: `TC(5) | EC(3) | C1..C8` (Junzi Sun guide, Chapter 2).
/// With this file’s ME convention (ME bit 1 = MSB of `me`), EC is ME bits 6–8.
pub fn parseAircraftIdentificationEmitterCategoryEc(me: u56, tc: u32) ?u3 {
    if (tc < 1 or tc > 4) return null;
    // ME bits 6–8 are the low 3 bits of the first ME byte (after the 5-bit TC).
    return @truncate((me >> 48) & 0x7);
}

/// True when DF17/18 extended squitter uses the 56-bit ME field for decoded content (callsign, position, velocity, etc.).
pub fn typeCodeUsesMe(tc: u32) bool {
    return (tc >= 1 and tc <= 18) or (tc >= 20 and tc <= 22) or tc == 19 or tc == 28 or tc == 31;
}

/// ME bit indices are **1-based**, MSB of `me` is ME bit 1.
fn meGetBitsInclusive1(me: u56, first: u6, last: u6) u32 {
    const len: u6 = last - first + 1;
    const shift: u6 = 56 - last;
    const mask: u56 = @truncate((@as(u64, 1) << len) - 1);
    return @truncate((me >> shift) & mask);
}

fn meGetBit1(me: u56, bit: u6) u1 {
    return @truncate((me >> @as(u6, 56 - bit)) & 1);
}

fn isAirbornePositionTypeCode(tc: u32) bool {
    return (tc >= 9 and tc <= 18) or (tc >= 20 and tc <= 22);
}

/// TC 9-18 use baro alt in the 12-bit field; TC 20-22 use GNSS / HAE-style height.
/// Both use the same 12-bit encoding family in DO-260.
pub fn airbornePositionIsBaroAltitude(tc: u32) bool {
    return tc >= 9 and tc <= 18;
}

test "parseAircraftIdentificationEmitterCategoryEc extracts ME bits 6-8" {
    // Construct an ME where TC=4 and EC=5 (binary 101), rest zero.
    // ME bit 1..5 = TC, bit 6..8 = EC. With ME MSB at bit 1:
    // me = (TC << 51) | (EC << 48).
    const tc: u32 = 4;
    const ec: u3 = 5;
    const me: u56 = (@as(u56, tc) << 51) | (@as(u56, ec) << 48);
    try std.testing.expectEqual(@as(?u3, ec), parseAircraftIdentificationEmitterCategoryEc(me, tc));
    try std.testing.expectEqual(@as(?u3, null), parseAircraftIdentificationEmitterCategoryEc(me, 0));
    try std.testing.expectEqual(@as(?u3, null), parseAircraftIdentificationEmitterCategoryEc(me, 9));
}

/// Reorder 13-bit field into hex Gillham layout.
pub fn decodeId13FieldGillham(id13: u32) u32 {
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

/// Mode A Gillham -> Mode C altitude in 100 ft units.
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

/// 13-bit identity field (TC 28 squawk, DF 5, etc.) → 4-digit squawk 0000–7777 (octal digits).
pub fn squawkDecimalFromId13(id13: u32) ?u16 {
    if (id13 == 0) return null;
    const g = decodeId13FieldGillham(id13 & 0x1FFF);
    const a = (g >> 12) & 7;
    const b = (g >> 8) & 7;
    const c = (g >> 4) & 7;
    const d = g & 7;
    const v: u32 = @as(u32, a) * 1000 + @as(u32, b) * 100 + @as(u32, c) * 10 + @as(u32, d);
    return @intCast(v);
}

/// TC 28 subtype 1 squawk; caller must only invoke when `tc == 28` (avoids redundant checks on hot path).
pub fn parseTc28EmergencySquawkMe(me: u56) ?u16 {
    const mesub = (me >> 48) & 0x7;
    if (mesub != 1) return null;
    const id13: u32 = @truncate((me >> 32) & 0x1FFF);
    return squawkDecimalFromId13(id13);
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

/// TC 9–18 (baro alt) and TC 20–22 (GNSS height): same ME layout as airborne CPR position.
pub fn parseAirbornePosition(me: u56, tc: u32) ?AirbornePosition {
    if (tc == 0) return null;
    if (!isAirbornePositionTypeCode(tc)) return null;

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

/// TC 31 Aircraft Operational Status for ADS-B versions 1-2.
pub const OperationalStatus = struct {
    /// ME bits 6–8: 0 airborne status, 1 surface status.
    mesub: u3,
    /// ME bits 41–43: ADS-B version number.
    version: u3,
    /// Version 1–2: NIC supplement A (ME bit 44).
    nic_a: ?u1 = null,
    /// Version 2 surface: NIC supplement C (ME bit 20) when `mesub == 1` and ME bits 9–10 are 0.
    nic_c: ?u1 = null,
    /// Version 1–2: NACp (ME bits 45–48).
    nac_p: ?u4 = null,
    /// Version 1–2: SIL (ME bits 51–52).
    sil: ?u2 = null,
    /// Version 2 only: ME bit 55 - `true` = per-sample integrity, `false` = per hour.
    sil_per_sample: ?bool = null,
    /// Version 2 airborne: geometric vertical accuracy GVA (ME bits 49–50).
    gva: ?u2 = null,
    /// Version 1–2 airborne: barometric altitude integrity / NIC baro (ME bit 53).
    nic_baro: ?u1 = null,
    /// Version 2 surface: NACv (ME bits 17–19) when `mesub == 1` and ME bits 9–10 are 0.
    nac_v_surface: ?u3 = null,
};

/// `tc` must be 31; returns null if ME subtype is not 0 or 1.
pub fn parseOperationalStatus(me: u56, tc: u32) ?OperationalStatus {
    if (tc != 31) return null;
    const mesub: u3 = @truncate(meGetBitsInclusive1(me, 6, 8));
    if (mesub != 0 and mesub != 1) return null;

    const version: u3 = @truncate(meGetBitsInclusive1(me, 41, 43));
    var out: OperationalStatus = .{
        .mesub = mesub,
        .version = version,
    };

    switch (version) {
        1 => {
            out.nic_a = @truncate(meGetBit1(me, 44));
            out.nac_p = @truncate(meGetBitsInclusive1(me, 45, 48));
            out.sil = @truncate(meGetBitsInclusive1(me, 51, 52));
            if (mesub == 0) {
                out.nic_baro = @truncate(meGetBit1(me, 53));
            }
        },
        2 => {
            out.nic_a = @truncate(meGetBit1(me, 44));
            out.nac_p = @truncate(meGetBitsInclusive1(me, 45, 48));
            out.sil = @truncate(meGetBitsInclusive1(me, 51, 52));
            out.sil_per_sample = meGetBit1(me, 55) != 0;
            if (mesub == 0) {
                out.gva = @truncate(meGetBitsInclusive1(me, 49, 50));
                out.nic_baro = @truncate(meGetBit1(me, 53));
            } else {
                if (meGetBitsInclusive1(me, 9, 10) == 0) {
                    out.nac_v_surface = @truncate(meGetBitsInclusive1(me, 17, 19));
                    out.nic_c = @truncate(meGetBit1(me, 20));
                }
            }
        },
        else => {},
    }
    return out;
}

/// TC 5-8 surface position ME layout.
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

/// ADS-B movement field v0 (ground speed kt, midpoint mapping).
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
            const src: []const u8 = if (airbornePositionIsBaroAltitude(tc)) "baro" else "gnss";
            std.debug.print("  alt: {d} ft ({s}, {s})\n", .{ alt_ft, src, if (q) "Q" else "Gillham" });
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

    if (tc == 31) {
        if (parseOperationalStatus(me, tc)) |op| {
            std.debug.print("  opstatus: ver={d} sub={d}", .{ op.version, op.mesub });
            if (op.nic_a) |v| std.debug.print(" NICa={d}", .{v});
            if (op.nic_c) |v| std.debug.print(" NICc={d}", .{v});
            if (op.nac_p) |v| std.debug.print(" NACp={d}", .{v});
            if (op.sil) |v| std.debug.print(" SIL={d}", .{v});
            if (op.sil_per_sample) |v| std.debug.print(" SILtp={s}", .{if (v) "sample" else "hour"});
            if (op.gva) |v| std.debug.print(" GVA={d}", .{v});
            if (op.nic_baro) |v| std.debug.print(" NICbaro={d}", .{v});
            if (op.nac_v_surface) |v| std.debug.print(" NACv_surf={d}", .{v});
            std.debug.print("\n", .{});
        }
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

    if (tc == 28) {
        const mesub = (me >> 48) & 0x7;
        if (mesub == 1) {
            const em = (me >> 45) & 0x7;
            const id13: u32 = @truncate((me >> 32) & 0x1FFF);
            if (squawkDecimalFromId13(id13)) |sq| {
                std.debug.print("  TC28 status: emergency={d} squawk={d:0>4}\n", .{ em, sq });
            } else {
                std.debug.print("  TC28 status: emergency={d} (squawk field 0)\n", .{em});
            }
        }
    }
}

test "typeCodeUsesMe" {
    try std.testing.expect(typeCodeUsesMe(4));
    try std.testing.expect(typeCodeUsesMe(11));
    try std.testing.expect(typeCodeUsesMe(19));
    try std.testing.expect(typeCodeUsesMe(20));
    try std.testing.expect(typeCodeUsesMe(22));
    try std.testing.expect(typeCodeUsesMe(28));
    try std.testing.expect(typeCodeUsesMe(31));
    try std.testing.expect(!typeCodeUsesMe(0));
    try std.testing.expect(!typeCodeUsesMe(29));
    try std.testing.expect(!typeCodeUsesMe(23));
}

test "parseAirbornePosition gnss tc22 same layout as baro tc11" {
    const tc_baro: u32 = 11;
    const tc_gnss: u32 = 22;
    // Minimal ME: only TC differs at top; rest zeros.
    const me_baro: u56 = @as(u56, tc_baro) << 51;
    const me_gnss: u56 = @as(u56, tc_gnss) << 51;
    const p1 = parseAirbornePosition(me_baro, tc_baro).?;
    const p2 = parseAirbornePosition(me_gnss, tc_gnss).?;
    try std.testing.expect(p1.lat_cpr == p2.lat_cpr and p1.lon_cpr == p2.lon_cpr);
    try std.testing.expect(airbornePositionIsBaroAltitude(tc_baro));
    try std.testing.expect(!airbornePositionIsBaroAltitude(tc_gnss));
}
