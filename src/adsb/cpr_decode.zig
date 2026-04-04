//! Compact Position Reporting (CPR) decode for ICAO 1090 MHz extended squitter.
//! Formulas and NL(lat) ladder follow the ADS-B MOPS (e.g. 1090-WP-9-14 / DO-260 family).
//! Results align with common open decoders (dump1090, readsb) used as interoperability checks.

const std = @import("std");

fn cprModInt(a: i32, b: i32) i32 {
    var res = @rem(a, b);
    if (res < 0) res += b;
    return res;
}

fn cprModDouble(a: f64, b: f64) f64 {
    var res = @mod(a, b);
    if (res < 0) res += b;
    return res;
}

/// NL(lat) from 1090-WP-9-14 (symmetric |lat|); piecewise structure matches the standard’s latitude zones.
fn nlL60(lat: f64) i32 {
    if (lat < 61.049_177_74) return 29;
    if (lat < 62.132_166_59) return 28;
    if (lat < 63.204_274_79) return 27;
    if (lat < 64.266_165_23) return 26;
    if (lat < 65.318_453_10) return 25;
    if (lat < 66.361_710_08) return 24;
    if (lat < 67.396_467_74) return 23;
    if (lat < 68.423_220_22) return 22;
    if (lat < 69.442_426_31) return 21;
    if (lat < 70.454_510_75) return 20;
    if (lat < 71.459_864_73) return 19;
    if (lat < 72.458_845_45) return 18;
    if (lat < 73.451_774_42) return 17;
    if (lat < 74.438_934_16) return 16;
    if (lat < 75.420_562_57) return 15;
    if (lat < 76.396_843_91) return 14;
    if (lat < 77.367_894_61) return 13;
    if (lat < 78.333_740_83) return 12;
    if (lat < 79.294_282_25) return 11;
    if (lat < 80.249_232_13) return 10;
    if (lat < 81.198_013_49) return 9;
    if (lat < 82.139_569_81) return 8;
    if (lat < 83.071_994_45) return 7;
    if (lat < 83.991_735_63) return 6;
    if (lat < 84.891_661_91) return 5;
    if (lat < 85.755_416_21) return 4;
    if (lat < 86.535_369_98) return 3;
    if (lat < 87.000_000_00) return 2;
    return 1;
}

fn nlL442ThenL60(lat: f64) i32 {
    if (lat < 45.546_267_23) return 42;
    if (lat < 46.867_332_52) return 41;
    if (lat < 48.160_391_28) return 40;
    if (lat < 49.427_764_39) return 39;
    if (lat < 50.671_501_66) return 38;
    if (lat < 51.893_424_69) return 37;
    if (lat < 53.095_161_53) return 36;
    if (lat < 54.278_174_72) return 35;
    if (lat < 55.443_784_44) return 34;
    if (lat < 56.593_187_56) return 33;
    if (lat < 57.727_473_54) return 32;
    if (lat < 58.847_637_76) return 31;
    if (lat < 59.954_592_77) return 30;
    return nlL60(lat);
}

fn nlL30ThenL442ThenL60(lat: f64) i32 {
    if (lat < 31.772_097_08) return 51;
    if (lat < 33.539_934_36) return 50;
    if (lat < 35.228_995_98) return 49;
    if (lat < 36.850_251_08) return 48;
    if (lat < 38.412_418_92) return 47;
    if (lat < 39.922_566_84) return 46;
    if (lat < 41.386_518_32) return 45;
    if (lat < 42.809_140_12) return 44;
    if (lat < 44.194_549_51) return 43;
    return nlL442ThenL60(lat);
}

pub fn cprNlFunction(lat_in: f64) i32 {
    var lat = lat_in;
    if (lat < 0) lat = -lat;
    if (lat > 60.0) return nlL60(lat);
    if (lat > 44.2) return nlL442ThenL60(lat);
    if (lat > 30.0) return nlL30ThenL442ThenL60(lat);
    if (lat < 10.470_471_30) return 59;
    if (lat < 14.828_174_37) return 58;
    if (lat < 18.186_263_57) return 57;
    if (lat < 21.029_394_93) return 56;
    if (lat < 23.545_044_87) return 55;
    if (lat < 25.829_247_07) return 54;
    if (lat < 27.938_987_10) return 53;
    if (lat < 29.911_356_86) return 52;
    return nlL30ThenL442ThenL60(lat);
}

fn cprNFunction(lat: f64, fflag: bool) i32 {
    var nl = cprNlFunction(lat) - @as(i32, if (fflag) 1 else 0);
    if (nl < 1) nl = 1;
    return nl;
}

fn cprDlonFunction(lat: f64, fflag: bool) f64 {
    return 360.0 / @as(f64, @floatFromInt(cprNFunction(lat, fflag)));
}

fn cprDlonSurface(lat: f64, fflag: bool) f64 {
    return 90.0 / @as(f64, @floatFromInt(cprNFunction(lat, fflag)));
}

const CPR_SCALE: f64 = 131072.0;

/// `fflag_odd`: when true, decode using the odd CPR frame (ME “F” / odd-subframe branch).
pub fn decodeCprAirborne(
    even_lat: u32,
    even_lon: u32,
    odd_lat: u32,
    odd_lon: u32,
    fflag_odd: bool,
) ?struct { lat: f64, lon: f64 } {
    const lat0: f64 = @floatFromInt(even_lat);
    const lat1: f64 = @floatFromInt(odd_lat);
    const lon0: f64 = @floatFromInt(even_lon);
    const lon1: f64 = @floatFromInt(odd_lon);

    const air_dlat0 = 360.0 / 60.0;
    const air_dlat1 = 360.0 / 59.0;

    const j: i32 = @intFromFloat(std.math.floor(((59.0 * lat0 - 60.0 * lat1) / CPR_SCALE) + 0.5));
    var rlat0 = air_dlat0 * (@as(f64, @floatFromInt(cprModInt(j, 60))) + lat0 / CPR_SCALE);
    var rlat1 = air_dlat1 * (@as(f64, @floatFromInt(cprModInt(j, 59))) + lat1 / CPR_SCALE);

    if (rlat0 >= 270.0) rlat0 -= 360.0;
    if (rlat1 >= 270.0) rlat1 -= 360.0;

    if (rlat0 < -90.0 or rlat0 > 90.0 or rlat1 < -90.0 or rlat1 > 90.0) return null;
    if (cprNlFunction(rlat0) != cprNlFunction(rlat1)) return null;

    var rlat: f64 = undefined;
    var rlon: f64 = undefined;

    if (fflag_odd) {
        const nl = cprNlFunction(rlat1);
        const ni = cprNFunction(rlat1, true);
        const m: i32 = @intFromFloat(std.math.floor((((lon0 * @as(f64, @floatFromInt(nl - 1))) - (lon1 * @as(f64, @floatFromInt(nl)))) / CPR_SCALE) + 0.5));
        rlon = cprDlonFunction(rlat1, true) * (@as(f64, @floatFromInt(cprModInt(m, ni))) + lon1 / CPR_SCALE);
        rlat = rlat1;
    } else {
        const nl = cprNlFunction(rlat0);
        const ni = cprNFunction(rlat0, false);
        const m: i32 = @intFromFloat(std.math.floor((((lon0 * @as(f64, @floatFromInt(nl - 1))) - (lon1 * @as(f64, @floatFromInt(nl)))) / CPR_SCALE) + 0.5));
        rlon = cprDlonFunction(rlat0, false) * (@as(f64, @floatFromInt(cprModInt(m, ni))) + lon0 / CPR_SCALE);
        rlat = rlat0;
    }

    rlon -= std.math.floor((rlon + 180.0) / 360.0) * 360.0;
    return .{ .lat = rlat, .lon = rlon };
}

pub fn decodeCprAirborneRelative(
    ref_lat: f64,
    ref_lon: f64,
    cpr_lat: u32,
    cpr_lon: u32,
    fflag: bool,
) ?struct { lat: f64, lon: f64 } {
    const fractional_lat = @as(f64, @floatFromInt(cpr_lat)) / CPR_SCALE;
    const fractional_lon = @as(f64, @floatFromInt(cpr_lon)) / CPR_SCALE;

    const lat_div: f64 = if (fflag) 59.0 else 60.0;
    const air_dlat = 360.0 / lat_div;

    const j: i32 = @intFromFloat(std.math.floor(ref_lat / air_dlat) +
        std.math.floor(0.5 + cprModDouble(ref_lat, air_dlat) / air_dlat - fractional_lat));
    var rlat = air_dlat * (@as(f64, @floatFromInt(j)) + fractional_lat);
    if (rlat >= 270.0) rlat -= 360.0;
    if (rlat < -90.0 or rlat > 90.0) return null;
    if (@abs(rlat - ref_lat) > air_dlat / 2.0) return null;

    const air_dlon = cprDlonFunction(rlat, fflag);
    const m: i32 = @intFromFloat(std.math.floor(ref_lon / air_dlon) +
        std.math.floor(0.5 + cprModDouble(ref_lon, air_dlon) / air_dlon - fractional_lon));
    var rlon = air_dlon * (@as(f64, @floatFromInt(m)) + fractional_lon);
    if (rlon > 180.0) rlon -= 360.0;
    if (@abs(rlon - ref_lon) > air_dlon / 2.0) return null;

    return .{ .lat = rlat, .lon = rlon };
}

/// Global surface CPR pair; `ref_lat`/`ref_lon` disambiguate quadrants per ICAO surface CPR rules.
pub fn decodeCprSurface(
    ref_lat: f64,
    ref_lon: f64,
    even_lat: u32,
    even_lon: u32,
    odd_lat: u32,
    odd_lon: u32,
    fflag_odd: bool,
) ?struct { lat: f64, lon: f64 } {
    const lat0: f64 = @floatFromInt(even_lat);
    const lat1: f64 = @floatFromInt(odd_lat);
    const lon0: f64 = @floatFromInt(even_lon);
    const lon1: f64 = @floatFromInt(odd_lon);

    const air_dlat0 = 90.0 / 60.0;
    const air_dlat1 = 90.0 / 59.0;

    const j: i32 = @intFromFloat(std.math.floor(((59.0 * lat0 - 60.0 * lat1) / CPR_SCALE) + 0.5));
    var rlat0 = air_dlat0 * (@as(f64, @floatFromInt(cprModInt(j, 60))) + lat0 / CPR_SCALE);
    var rlat1 = air_dlat1 * (@as(f64, @floatFromInt(cprModInt(j, 59))) + lat1 / CPR_SCALE);

    if (rlat0 == 0.0) {
        if (ref_lat < -45.0) {
            rlat0 = -90.0;
        } else if (ref_lat > 45.0) {
            rlat0 = 90.0;
        }
    } else if ((rlat0 - ref_lat) > 45.0) {
        rlat0 -= 90.0;
    }

    if (rlat1 == 0.0) {
        if (ref_lat < -45.0) {
            rlat1 = -90.0;
        } else if (ref_lat > 45.0) {
            rlat1 = 90.0;
        }
    } else if ((rlat1 - ref_lat) > 45.0) {
        rlat1 -= 90.0;
    }

    if (rlat0 < -90.0 or rlat0 > 90.0 or rlat1 < -90.0 or rlat1 > 90.0) return null;
    if (cprNlFunction(rlat0) != cprNlFunction(rlat1)) return null;

    var rlat: f64 = undefined;
    var rlon: f64 = undefined;

    if (fflag_odd) {
        const nl = cprNlFunction(rlat1);
        const ni = cprNFunction(rlat1, true);
        const m: i32 = @intFromFloat(std.math.floor((((lon0 * @as(f64, @floatFromInt(nl - 1))) - (lon1 * @as(f64, @floatFromInt(nl)))) / CPR_SCALE) + 0.5));
        rlon = cprDlonSurface(rlat1, true) * (@as(f64, @floatFromInt(cprModInt(m, ni))) + lon1 / CPR_SCALE);
        rlat = rlat1;
    } else {
        const nl = cprNlFunction(rlat0);
        const ni = cprNFunction(rlat0, false);
        const m: i32 = @intFromFloat(std.math.floor((((lon0 * @as(f64, @floatFromInt(nl - 1))) - (lon1 * @as(f64, @floatFromInt(nl)))) / CPR_SCALE) + 0.5));
        rlon = cprDlonSurface(rlat0, false) * (@as(f64, @floatFromInt(cprModInt(m, ni))) + lon0 / CPR_SCALE);
        rlat = rlat0;
    }

    rlon += std.math.floor((ref_lon - rlon + 45.0) / 90.0) * 90.0;
    rlon -= std.math.floor((rlon + 180.0) / 360.0) * 360.0;

    return .{ .lat = rlat, .lon = rlon };
}

pub fn decodeCprSurfaceRelative(
    ref_lat: f64,
    ref_lon: f64,
    cpr_lat: u32,
    cpr_lon: u32,
    fflag: bool,
) ?struct { lat: f64, lon: f64 } {
    const fractional_lat = @as(f64, @floatFromInt(cpr_lat)) / CPR_SCALE;
    const fractional_lon = @as(f64, @floatFromInt(cpr_lon)) / CPR_SCALE;

    const lat_div: f64 = if (fflag) 59.0 else 60.0;
    const air_dlat = 90.0 / lat_div;

    const j: i32 = @intFromFloat(std.math.floor(ref_lat / air_dlat) +
        std.math.floor(0.5 + cprModDouble(ref_lat, air_dlat) / air_dlat - fractional_lat));
    var rlat = air_dlat * (@as(f64, @floatFromInt(j)) + fractional_lat);
    if (rlat >= 270.0) rlat -= 360.0;
    if (rlat < -90.0 or rlat > 90.0) return null;
    if (@abs(rlat - ref_lat) > air_dlat / 2.0) return null;

    const air_dlon = cprDlonSurface(rlat, fflag);
    const m: i32 = @intFromFloat(std.math.floor(ref_lon / air_dlon) +
        std.math.floor(0.5 + cprModDouble(ref_lon, air_dlon) / air_dlon - fractional_lon));
    var rlon = air_dlon * (@as(f64, @floatFromInt(m)) + fractional_lon);
    if (rlon > 180.0) rlon -= 360.0;
    if (@abs(rlon - ref_lon) > air_dlon / 2.0) return null;

    return .{ .lat = rlat, .lon = rlon };
}
