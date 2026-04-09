//! Mode S Manchester demodulation at 1 Mbit/s with variable sample rate.
//! Preamble and chip timing use a 2.0 Msps reference scaling commonly used by Mode S decoders.

const std = @import("std");
const C32 = std.math.Complex(f32);

pub const DemodParams = struct {
    sample_rate_hz: f64,
    /// Samples per 0.5 µs half-chip (Mode S = 1 MHz Manchester).
    samples_per_half_chip: f64,
    preamble_samples: usize,
    pulse_s: [4]usize,
    gap_s: [8]usize,

    pub fn init(sample_rate_hz: f64) DemodParams {
        const sr = sample_rate_hz;
        // 2 Msps reference: 1 sample per half-chip.
        const sphc = sr / 2e6;
        const pulse_us = [_]f64{ 0.0, 1.0, 3.5, 4.5 };
        var pulse_s: [4]usize = undefined;
        for (pulse_us, 0..) |pu, k| {
            pulse_s[k] = @max(0, @as(usize, @intFromFloat(@round(pu * sr / 1e6))));
        }
        const base_gaps: [8]usize = .{ 1, 3, 4, 5, 6, 8, 10, 11 };
        const scale = sr / 2e6;
        var gap_s: [8]usize = undefined;
        for (base_gaps, 0..) |g, k| {
            gap_s[k] = @max(0, @as(usize, @intFromFloat(@round(@as(f64, @floatFromInt(g)) * scale))));
        }
        const preamble_samples = @max(1, @as(usize, @intFromFloat(@ceil(8e-6 * sr))));
        return .{
            .sample_rate_hz = sr,
            .samples_per_half_chip = sphc,
            .preamble_samples = preamble_samples,
            .pulse_s = pulse_s,
            .gap_s = gap_s,
        };
    }

    /// Samples from preamble start through end of `bit_len` Manchester bits.
    pub fn messageExtentSamples(self: DemodParams, bit_len: usize) usize {
        const data = @ceil(@as(f64, @floatFromInt(bit_len * 2)) * self.samples_per_half_chip);
        return self.preamble_samples + @as(usize, @intFromFloat(data)) + 2;
    }
};

pub fn energyAt(energy: []const f32, idx: f64) f32 {
    if (energy.len == 0) return 0;
    if (idx <= 0) return energy[0];
    const last = @as(f64, @floatFromInt(energy.len - 1));
    if (idx >= last) return energy[energy.len - 1];
    const i_floor = @floor(idx);
    const frac = idx - i_floor;
    const base: usize = @intFromFloat(i_floor);
    const e0: f32 = energy[base];
    if (frac == 0.0) return e0;
    const e1: f32 = energy[base + 1];
    const f: f32 = @floatCast(frac);
    return e0 * (1.0 - f) + e1 * f;
}

pub fn sampleAt(samples: []const C32, idx: f64) C32 {
    if (samples.len == 0) return .{ .re = 0.0, .im = 0.0 };
    if (idx <= 0.0) return samples[0];
    const last = @as(f64, @floatFromInt(samples.len - 1));
    if (idx >= last) return samples[samples.len - 1];
    const i_floor = @floor(idx);
    const frac = idx - i_floor;
    const base: usize = @intFromFloat(i_floor);
    const s0 = samples[base];
    if (frac == 0.0) return s0;
    const s1 = samples[base + 1];
    const f: f32 = @floatCast(frac);
    return .{
        .re = s0.re * (1.0 - f) + s1.re * f,
        .im = s0.im * (1.0 - f) + s1.im * f,
    };
}

fn norm2(s: C32) f32 {
    return s.re * s.re + s.im * s.im;
}

/// Estimate a unit phase reference from preamble pulse samples.
pub fn estimatePreamblePhaseRef(samples: []const C32, i: usize, params: DemodParams) ?C32 {
    if (i + params.preamble_samples > samples.len) return null;
    var sum_re: f32 = 0.0;
    var sum_im: f32 = 0.0;
    for (params.pulse_s) |p| {
        if (i + p >= samples.len) return null;
        const s = samples[i + p];
        const mag2 = norm2(s);
        if (mag2 <= 1e-12) continue;
        const inv_mag = 1.0 / @sqrt(mag2);
        sum_re += s.re * inv_mag;
        sum_im += s.im * inv_mag;
    }
    const vmag2 = sum_re * sum_re + sum_im * sum_im;
    if (vmag2 <= 1e-12) return null;
    const inv_vmag = 1.0 / @sqrt(vmag2);
    return .{ .re = sum_re * inv_vmag, .im = sum_im * inv_vmag };
}

fn chipMetric(e: f32, s: C32, phase_ref: ?C32, phase_weight: f32) f32 {
    if (phase_ref == null or phase_weight <= 0.0) return e;
    const pr = phase_ref.?;
    const coh = s.re * pr.re + s.im * pr.im;
    const coh_pow = coh * coh;
    const w = std.math.clamp(phase_weight, 0.0, 1.0);
    return e * (1.0 - w) + coh_pow * w;
}

pub fn likelyPreamble(
    energy: []const f32,
    i: usize,
    noise_floor: f32,
    params: DemodParams,
) bool {
    return preambleScore(energy, i, noise_floor, params) > 2.0;
}

/// Returns a quality score for a potential preamble candidate.
/// Higher scores indicate stronger pulse/gap contrast and better SNR.
pub fn preambleScore(
    energy: []const f32,
    i: usize,
    noise_floor: f32,
    params: DemodParams,
) f32 {
    if (i + params.preamble_samples > energy.len) return 0.0;

    // Hot fast-path: 2.0 Msps has fixed preamble sample offsets.
    if (@abs(params.samples_per_half_chip - 1.0) < 1e-12) {
        // Pulses at 0, 1.0us, 3.5us, 4.5us -> samples 0,2,7,9 at 2 Msps.
        const p0 = energy[i + 0];
        const p1 = energy[i + 2];
        const p2 = energy[i + 7];
        const p3 = energy[i + 9];
        const pulse_avg: f32 = (p0 + p1 + p2 + p3) * 0.25;
        if (pulse_avg < noise_floor * 3.0) return 0.0;

        // Gaps at 0.5us, 1.5us, 2.0us, 2.5us, 3.0us, 4.0us, 5.0us, 5.5us
        // -> samples 1,3,4,5,6,8,10,11 at 2 Msps.
        const g0 = energy[i + 1];
        const g1 = energy[i + 3];
        const g2 = energy[i + 4];
        const g3 = energy[i + 5];
        const g4 = energy[i + 6];
        const g5 = energy[i + 8];
        const g6 = energy[i + 10];
        const g7 = energy[i + 11];
        const gap_avg: f32 = (g0 + g1 + g2 + g3 + g4 + g5 + g6 + g7) * 0.125;
        if (pulse_avg <= gap_avg * 2.0) return 0.0;
        const snr_ratio = pulse_avg / @max(1e-6, noise_floor);
        const contrast = pulse_avg / @max(1e-6, gap_avg);
        return (snr_ratio - 3.0) + (contrast - 2.0);
    }

    var pulse_sum: f32 = 0;
    for (params.pulse_s) |p| {
        if (i + p >= energy.len) return 0.0;
        pulse_sum += energy[i + p];
    }
    const pulse_avg: f32 = pulse_sum / 4.0;
    if (pulse_avg < noise_floor * 3.0) return 0.0;

    var gap_sum: f32 = 0;
    for (params.gap_s) |g| {
        if (i + g >= energy.len) return 0.0;
        gap_sum += energy[i + g];
    }
    const gap_avg: f32 = gap_sum / 8.0;
    if (pulse_avg <= gap_avg * 2.0) return 0.0;
    const snr_ratio = pulse_avg / @max(1e-6, noise_floor);
    const contrast = pulse_avg / @max(1e-6, gap_avg);
    return (snr_ratio - 3.0) + (contrast - 2.0);
}

/// Decode with a bounded timing search around `data_start`.
/// Returns best confidence observed and writes best bits into `bits`.
pub fn decodeBitsManchesterBestOffset(
    energy: []const f32,
    data_start: f64,
    bits: []u1,
    scratch: []u1,
    samples_per_half_chip: f64,
    offsets: []const f64,
) f32 {
    if (offsets.len == 0) {
        return decodeBitsManchester(energy, data_start, bits, samples_per_half_chip);
    }
    var best_conf: f32 = -1.0;
    for (offsets) |off| {
        const conf = decodeBitsManchester(energy, data_start + off, scratch, samples_per_half_chip);
        if (conf > best_conf) {
            best_conf = conf;
            @memcpy(bits, scratch);
        }
    }
    return best_conf;
}

/// Decode with timing search and optional phase-enhanced chip metric.
pub fn decodeBitsManchesterBestOffsetPhaseEnhanced(
    energy: []const f32,
    samples: []const C32,
    data_start: f64,
    bits: []u1,
    scratch: []u1,
    samples_per_half_chip: f64,
    offsets: []const f64,
    phase_ref: ?C32,
    phase_weight: f32,
) f32 {
    if (phase_ref == null or phase_weight <= 0.0) {
        return decodeBitsManchesterBestOffset(energy, data_start, bits, scratch, samples_per_half_chip, offsets);
    }
    if (offsets.len == 0) {
        return decodeBitsManchesterPhaseEnhanced(energy, samples, data_start, bits, samples_per_half_chip, phase_ref, phase_weight);
    }
    var best_conf: f32 = -1.0;
    for (offsets) |off| {
        const conf = decodeBitsManchesterPhaseEnhanced(
            energy,
            samples,
            data_start + off,
            scratch,
            samples_per_half_chip,
            phase_ref,
            phase_weight,
        );
        if (conf > best_conf) {
            best_conf = conf;
            @memcpy(bits, scratch);
        }
    }
    return best_conf;
}

/// 2.0 Msps integer sample indices: same chipMetric as slow path, no interpolation.
fn decodeBitsManchesterPhaseEnhancedIntSampleIndex(
    energy: []const f32,
    samples: []const C32,
    start_i: usize,
    bits: []u1,
    phase_ref: ?C32,
    phase_weight: f32,
) f32 {
    var confidence: f32 = 0.0;
    for (bits, 0..) |_, b| {
        const e0 = energy[start_i + b * 2];
        const e1 = energy[start_i + b * 2 + 1];
        const s0 = samples[start_i + b * 2];
        const s1 = samples[start_i + b * 2 + 1];
        const m0 = chipMetric(e0, s0, phase_ref, phase_weight);
        const m1 = chipMetric(e1, s1, phase_ref, phase_weight);
        bits[b] = if (m0 > m1) 1 else 0;
        confidence += @abs(m0 - m1);
    }
    return confidence / @as(f32, @floatFromInt(bits.len));
}

/// Manchester decode using blended energy/coherent-phase chip metrics.
pub fn decodeBitsManchesterPhaseEnhanced(
    energy: []const f32,
    samples: []const C32,
    data_start: f64,
    bits: []u1,
    samples_per_half_chip: f64,
    phase_ref: ?C32,
    phase_weight: f32,
) f32 {
    if (phase_ref != null and phase_weight > 0.0 and bits.len > 0) {
        if (@abs(samples_per_half_chip - 1.0) < 1e-12) {
            const r = @round(data_start);
            if (@abs(data_start - r) < 1e-7) {
                const start_i: usize = @intFromFloat(r);
                if (start_i + 2 * bits.len <= energy.len and start_i + 2 * bits.len <= samples.len) {
                    return decodeBitsManchesterPhaseEnhancedIntSampleIndex(
                        energy,
                        samples,
                        start_i,
                        bits,
                        phase_ref,
                        phase_weight,
                    );
                }
            }
        }
    }
    var confidence: f32 = 0.0;
    for (bits, 0..) |_, b| {
        const chip_a = data_start + @as(f64, @floatFromInt(b * 2)) * samples_per_half_chip;
        const chip_b = data_start + @as(f64, @floatFromInt(b * 2 + 1)) * samples_per_half_chip;
        const e0 = energyAt(energy, chip_a);
        const e1 = energyAt(energy, chip_b);
        const s0 = sampleAt(samples, chip_a);
        const s1 = sampleAt(samples, chip_b);
        const m0 = chipMetric(e0, s0, phase_ref, phase_weight);
        const m1 = chipMetric(e1, s1, phase_ref, phase_weight);
        bits[b] = if (m0 > m1) 1 else 0;
        confidence += @abs(m0 - m1);
    }
    return confidence / @as(f32, @floatFromInt(bits.len));
}

/// Manchester decode `bits.len` bits starting at `data_start` (fractional sample index).
pub fn decodeBitsManchester(
    energy: []const f32,
    data_start: f64,
    bits: []u1,
    samples_per_half_chip: f64,
) f32 {
    // Fast path for 2.0 Msps where half-chip boundaries are exact samples.
    if (@abs(samples_per_half_chip - 1.0) < 1e-12) {
        const start_i: usize = @intFromFloat(data_start);
        var confidence: f32 = 0.0;
        for (bits, 0..) |_, b| {
            const e0 = energy[start_i + b * 2];
            const e1 = energy[start_i + b * 2 + 1];
            bits[b] = if (e0 > e1) 1 else 0;
            confidence += @abs(e0 - e1);
        }
        return confidence / @as(f32, @floatFromInt(bits.len));
    }

    var confidence: f32 = 0.0;
    for (bits, 0..) |_, b| {
        const chip_a = data_start + @as(f64, @floatFromInt(b * 2)) * samples_per_half_chip;
        const chip_b = data_start + @as(f64, @floatFromInt(b * 2 + 1)) * samples_per_half_chip;
        const e0 = energyAt(energy, chip_a);
        const e1 = energyAt(energy, chip_b);
        bits[b] = if (e0 > e1) 1 else 0;
        confidence += @abs(e0 - e1);
    }
    return confidence / @as(f32, @floatFromInt(bits.len));
}

pub fn bitsToBytes(bits: []const u1, out: []u8) void {
    @memset(out, 0);
    for (bits, 0..) |bit, idx| {
        if (bit == 1) {
            out[idx / 8] |= @as(u8, 1) << @intCast(7 - (idx % 8));
        }
    }
}
