const std = @import("std");
const Log = @import("Log").Log;
const LogConfig = @import("Log").Config;
const RSDepth = @import("rs-depth.zig").RealsenseDepth;
const RSConfig = @import("rs-depth.zig").Config;
const KalmanFilter = @import("KalmanFilter.zig").KalmanFilter(2, 1, 1);
const IKDTree = @import("IKDTree.zig").IKDTree;
const Slam = @This();

const I3DTree = IKDTree(3);
var log: Log = undefined;
var gpa = std.heap.GeneralPurposeAllocator(.{}){};
var allocator = gpa.allocator();
var kf: KalmanFilter = undefined;
var rs_depth: RSDepth = undefined;

pub const Config = struct {
    log_config: LogConfig,
    kalman_filter: KalmanFilter.Config,
    ikd_tree: I3DTree.Config,
    rs_config: RSConfig,
};

pub fn start() void {
    log.info("Starting", .{});
    rs_depth.start_loop();
}

pub fn stop() void {
    log.info("Stopping", .{});
    rs_depth.stop_loop();
}

pub fn init(config: Config) !void {
    log = try Log.init(config.log_config);
    log.info("Initializing", .{});

    var rand_default_prng = std.Random.DefaultPrng.init(0);
    const rand = rand_default_prng.random();

    const dim = 100000;
    var points = try allocator.alloc([3]f32, dim);
    for (0..dim) |k| {
        points[k] = .{
            @round(rand.float(f32) * 500) / 100,
            @round(rand.float(f32) * 500) / 100,
            @round(rand.float(f32) * 500) / 100,
        };
    }

    // Making a fixed buffer allocator makes the ikd tree much faster
    const buffer = try allocator.alloc(u8, 1 * try std.math.powi(usize, 2, 30));
    defer allocator.free(buffer);
    var fba = std.heap.FixedBufferAllocator.init(buffer);

    var timer = std.time.Timer.start() catch unreachable;
    var ikd = try I3DTree.init(points, config.ikd_tree, fba.allocator(), &log);
    log.debug("IKD tree init time {}", .{std.fmt.fmtDuration(timer.lap())});
    try ikd.insert(.{ 10, 10, 10 });

    allocator.free(points);
    defer ikd.deinit();
    //ikd.print();

    const query_point: [3]f32 = .{ 3, 3, 3 };
    _ = timer.lap();
    const nearest = ikd.nearestNeighbor(query_point).?;
    log.debug("IKD tree nnSearch time {}", .{std.fmt.fmtDuration(timer.lap())});
    log.info("Closest point to {d}: {d}", .{ query_point, nearest });

    _ = timer.lap();
    const knearest = try ikd.kNearestNeighbors(query_point, 5);
    log.debug("IKD tree knnSearch time {}", .{std.fmt.fmtDuration(timer.lap())});
    log.info("Closest points to {d}:", .{query_point});
    for (knearest) |k| {
        log.info("  {d}", .{k});
    }

    kf = try KalmanFilter.init(
        config.kalman_filter,
        &log,
        [1][2]f32{.{ 1, 0 }}, // H
        [1][1]f32{.{400}}, // R
        null, // x
        [2][2]f32{ .{ 1, 0.25 }, .{ 0, 1 } }, // F
        [2][1]f32{ .{0.0313}, .{0.25} }, // G
        [2][2]f32{ .{ 500, 0 }, .{ 0, 500 } }, // P
        [2][2]f32{ .{ 9.765625e-6, 7.8125e-3 }, .{ 7.8125e-3, 6.25e-2 } }, // Q
    );
    defer kf.deinit();

    const x = try kf.iterate(
        [1]f32{6.43}, // z
        null, // H
        null, // R
        [1]f32{39.81 - 9.81}, // u
        null, // F
        null, // G
        null, // Q
    );
    log.info("x_1: {d}", .{x});

    rs_depth = try RSDepth.init(config.rs_config);
}

pub fn deinit() void {
    rs_depth.deinit();

    log.info("Deinitializing", .{});
    log.deinit();

    if (gpa.deinit() == .leak) {
        log.err("Memory leaks detected", .{});
    }
}

test {}
