const std = @import("std");
pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const exe = b.addExecutable(.{
        .name = "zig1090",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    exe.linkLibC();

    exe.linkSystemLibrary("rtlsdr");

    switch (target.result.os.tag) {
        .linux => {
            exe.linkSystemLibrary("usb-1.0");
            exe.linkSystemLibrary("m");
            exe.linkSystemLibrary("pthread");
        },
        .macos => {
            exe.linkSystemLibrary("usb-1.0");
            exe.linkSystemLibrary("iconv");
        },
        .openbsd => {
            exe.linkSystemLibrary("usb-1.0");
        },
        .windows => {
            exe.addLibraryPath(.{ .cwd_relative = "vcpkg/installed/x64-windows/lib" });
            exe.addIncludePath(.{ .cwd_relative = "vcpkg/installed/x64-windows/include" });
        },
        else => {},
    }

    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    if (b.args) |args| run_cmd.addArgs(args);

    const run_step = b.step("run", "Run zig1090 ADS-B demodulator");
    run_step.dependOn(&run_cmd.step);
}
