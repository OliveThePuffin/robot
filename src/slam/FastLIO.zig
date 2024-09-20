const std = @import("std");
const Log = @import("Log").Log;
const Unit = @import("Unit").Unit;
const LogConfig = @import("Log").Config;
const KalmanFilter = @import("KalmanFilter.zig").KalmanFilter(2, 1, 1);
const IKDTree = @import("IKDTree.zig").IKDTree;
const IMU = @import("IMU.zig").IMU;

pub const FastLIO = struct {
    const Self = @This();
    const GPA = std.heap.GeneralPurposeAllocator(.{});

    log: Log,
    gpa: GPA,

    imu: Unit,
    // TODO: have an imu buffer that the IMU writes to
    kf: KalmanFilter,

    const I3DTree = IKDTree(3);

    pub const Config = struct {
        log: LogConfig,
        imu: IMU.Config,
        kalman_filter: KalmanFilter.Config,
        ikd_tree: I3DTree.Config,
        frequency: f32,
    };

    pub fn init(config: Config) !Unit {
        // Start threads:
        // Get IMU input (100-250Hz)
        // Get Lidar input (100k-500kHz)

        var gpa = GPA{};
        var log = try Log.init(config.log);
        errdefer {
            if (gpa.deinit() == .leak) {
                log.err("Memory leaks detected", .{});
            }
        }

        const self_ptr = try gpa.allocator().create(Self);
        self_ptr.* = FastLIO{
            .log = log,
            .gpa = gpa,
            .imu = try IMU.init(config.imu),
            .kf = undefined,
        };
        self_ptr.log.info("Initializing", .{});

        return Unit{
            .log = self_ptr.log,
            .frequency = config.frequency,
            .ctx = self_ptr,
            .fn_start = start,
            .fn_stop = stop,
            .fn_deinit = deinit,
            .fn_update = update,
        };
    }

    pub fn deinit(ctx: *anyopaque) void {
        var self: *Self = @ptrCast(@alignCast(ctx));
        self.log.info("Deinitializing", .{});
        self.imu.deinit();

        var log = self.log;
        var gpa = self.gpa;
        gpa.allocator().destroy(self);

        if (gpa.deinit() == .leak) {
            log.err("Memory leaks detected", .{});
        }
        log.deinit();
    }

    pub fn start(ctx: *anyopaque) void {
        var self: *Self = @ptrCast(@alignCast(ctx));
        self.imu.start();
    }

    pub fn stop(ctx: *anyopaque) void {
        var self: *Self = @ptrCast(@alignCast(ctx));
        self.imu.stop();
    }

    fn update(ctx: *anyopaque) !void {
        _ = ctx;
    }
};
