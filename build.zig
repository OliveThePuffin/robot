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
    const module_web = b.dependency("zap", .{ .target = target, .optimize = optimize, .openssl = false }).module("zap");
    const module_config = b.addModule("config", .{ .root_source_file = b.path("src/config/Config.zig") });
    const module_slam = b.addModule("Slam", .{ .root_source_file = b.path("src/slam/Slam.zig") });
    const module_ocl_helper = b.addModule("ocl_helper", .{ .root_source_file = b.path("src/opencl/cl_helper.zig") });
    const module_log = b.addModule("logger", .{ .root_source_file = b.path("src/logger.zig") });
    module_ocl_helper.addImport("opencl", module_cl);
    module_ocl_helper.addImport("logger", module_log);
    module_slam.addImport("ocl_helper", module_ocl_helper);
    module_slam.addImport("logger", module_log);

    //const lib = b.addStaticLibrary(.{
    //    .name = "robot",
    //    // In this case the main source file is merely a path, however, in more
    //    // complicated build scripts, this could be a generated file.
    //    .root_source_file = b.path("src/root.zig"),
    //    .target = target,
    //    .optimize = optimize,
    //});
    //lib.root_module.addImport("logger", module_log);

    // This declares intent for the library to be installed into the standard
    // location when the user invokes the "install" step (the default step when
    // running `zig build`).
    //b.installArtifact(lib);

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
    exe.root_module.addImport("logger", module_log);
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

    // Creates a step for unit testing. This only builds the test executable
    // but does not run it.
    //const lib_unit_tests = b.addTest(.{
    //    .root_source_file = b.path("src/root.zig"),
    //    .target = target,
    //    .optimize = optimize,
    //});

    //const run_lib_unit_tests = b.addRunArtifact(lib_unit_tests);

    const exe_unit_tests = .{
        b.addTest(.{ .root_source_file = b.path("src/main.zig") }),
        b.addTest(.{ .root_source_file = b.path("src/logger.zig") }),
        b.addTest(.{ .root_source_file = b.path("src/config/Config.zig") }),
        b.addTest(.{ .root_source_file = b.path("src/slam/Slam.zig") }),
        b.addTest(.{ .root_source_file = b.path("src/slam/KalmanFilter.zig") }),
        b.addTest(.{ .root_source_file = b.path("src/slam/IKDTree.zig") }),
    };
    inline for (exe_unit_tests) |exe_unit_test| {
        exe_unit_test.root_module.addImport("logger", module_log);
        exe_unit_test.linkLibC();
    }
    exe_unit_tests[4].root_module.addImport("ocl_helper", module_ocl_helper);

    // Similar to creating the run step earlier, this exposes a `test` step to
    // the `zig build --help` menu, providing a way for the user to request
    // running the unit tests.

    const test_step = b.step("test", "Run unit tests");
    inline for (exe_unit_tests) |exe_unit_test| {
        const run_exe_unit_test = b.addRunArtifact(exe_unit_test);
        test_step.dependOn(&run_exe_unit_test.step);
    }
}
