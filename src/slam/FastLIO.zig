const std = @import("std");
const Log = @import("Log").Log;
const Unit = @import("Unit").Unit;
const LogConfig = @import("Log").Config;
const RSDepth = @import("rs-depth.zig").RealsenseDepth;
const RSConfig = @import("rs-depth.zig").Config;
const KalmanFilter = @import("KalmanFilter.zig").KalmanFilter(2, 1, 1);
const IKDTree = @import("IKDTree.zig").IKDTree;

pub const FastLIO = struct {
    const Self = @This();
    const GPA = std.heap.GeneralPurposeAllocator(.{});

    log: Log,
    gpa: GPA,
    kf: KalmanFilter,
    rs_depth: RSDepth,

    const I3DTree = IKDTree(3);

    pub const Config = struct {
        log: LogConfig,
        kalman_filter: KalmanFilter.Config,
        ikd_tree: I3DTree.Config,
        rs_config: RSConfig,
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
            .kf = undefined,
            .rs_depth = try RSDepth.init(config.rs_config),
        };
        self_ptr.log.info("Initializing", .{});

        return Unit{
            .log = self_ptr.log,
            .frequency = config.frequency,
            .impl = self_ptr,
            .fn_deinit = deinit,
            .fn_update = update,
        };
    }

    pub fn deinit(ctx: *anyopaque) void {
        var self: *Self = @ptrCast(@alignCast(ctx));
        self.log.info("Deinitializing", .{});

        var log = self.log;
        var gpa = self.gpa;
        gpa.allocator().destroy(self);

        if (gpa.deinit() == .leak) {
            log.err("Memory leaks detected", .{});
        }
        log.deinit();
    }

    fn update(ctx: *anyopaque) !void {
        const self: *Self = @ptrCast(@alignCast(ctx));
        _ = self;
    }
};
