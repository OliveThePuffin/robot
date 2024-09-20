const std = @import("std");

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.
pub fn build(b: *std.Build) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    // Modules
    const module_cl = b.dependency("zig-opencl", .{ .target = target, .optimize = optimize }).module("opencl");
    const module_ocl_helper = b.addModule("ocl_helper", .{ .root_source_file = b.path("src/opencl/cl_helper.zig") });

    const module_web = b.dependency("zap", .{ .target = target, .optimize = optimize, .openssl = false }).module("zap");

    const module_slam = b.addModule("Slam", .{ .root_source_file = b.path("src/slam/FastLIO.zig") });

    const module_log = b.addModule("Log", .{ .root_source_file = b.path("src/Log.zig") });
    const module_unit = b.addModule("Unit", .{ .root_source_file = b.path("src/Unit.zig") });

    const module_config = b.addModule("config", .{ .root_source_file = b.path("src/config/Config.zig") });

    // Module deps
    module_ocl_helper.addImport("opencl", module_cl);
    module_ocl_helper.addImport("Log", module_log);
    module_unit.addImport("Log", module_log);
    module_slam.addImport("Log", module_log);
    module_slam.addImport("Unit", module_unit);
    module_config.addImport("Slam", module_slam);

    const exe = b.addExecutable(.{
        .name = "robot",
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    exe.root_module.addImport("zap", module_web);
    exe.root_module.addImport("Config", module_config);
    exe.root_module.addImport("Slam", module_slam);
    module_slam.addImport("ocl_helper", module_ocl_helper);
    exe.root_module.addImport("Log", module_log);
    exe.linkLibC();
    exe.linkSystemLibrary("realsense2");

    exe.addIncludePath(.{ .src_path = .{
        .owner = b,
        .sub_path = "/usr/local/include",
    } });

    // This declares intent for the executable to be installed into the
    // standard location when the user invokes the "install" step (the default
    // step when running `zig build`).
    b.installArtifact(exe);

    // This *creates* a Run step in the build graph, to be executed when another
    // step is evaluated that depends on it. The next line below will establish
    // such a dependency.
    const run_cmd = b.addRunArtifact(exe);

    // By making the run step depend on the install step, it will be run from the
    // installation directory rather than directly from within the cache directory.
    // This is not necessary, however, if the application depends on other installed
    // files, this ensures they will be present and in the expected location.
    run_cmd.step.dependOn(b.getInstallStep());

    // This allows the user to pass arguments to the application in the build
    // command itself, like this: `zig build run -- arg1 arg2 etc`
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    // This creates a build step. It will be visible in the `zig build --help` menu,
    // and can be selected like this: `zig build run`
    // This will evaluate the `run` step rather than the default, which is "install".
    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    // Unit tests
    const kf_test = b.addTest(.{
        .root_source_file = b.path("src/slam/KalmanFilter.zig"),
    });
    const unit_tests = .{
        b.addTest(.{ .root_source_file = b.path("src/main.zig") }),
        b.addTest(.{ .root_source_file = b.path("src/Log.zig") }),
        b.addTest(.{ .root_source_file = b.path("src/config/Config.zig") }),
        kf_test,
        b.addTest(.{ .root_source_file = b.path("src/slam/IKDTree.zig") }),
    };
    inline for (unit_tests) |unit_test| {
        unit_test.root_module.addImport("Log", module_log);
        unit_test.linkLibC();
    }
    kf_test.root_module.addImport("ocl_helper", module_ocl_helper);

    const test_step = b.step("test", "Run unit tests");
    inline for (unit_tests) |unit_test| {
        const run_unit_test = b.addRunArtifact(unit_test);
        test_step.dependOn(&run_unit_test.step);
    }

    // Performance tests
    const ikd_tree_perf = b.addExecutable(.{
        .name = "perf",
        .root_source_file = b.path("src/perf_root.zig"),
        .target = target,
        .optimize = optimize,
    });
    ikd_tree_perf.root_module.addImport("Log", module_log);
    b.installArtifact(ikd_tree_perf);
    const perf_cmd = b.addRunArtifact(ikd_tree_perf);
    perf_cmd.step.dependOn(b.getInstallStep());
    const perf_step = b.step("perf", "Run performance tests");
    perf_step.dependOn(&perf_cmd.step);
}
