const std = @import("std");
const Unit = @import("Unit").Unit;
const Log = @import("Log").Log;
const LogConfig = @import("Log").Config;

pub const IMU = struct {
    const Self = @This();
    const GPA = std.heap.GeneralPurposeAllocator(.{});

    log: Log,
    gpa: GPA,

    pub const Config = struct {
        log: LogConfig,
        frequency: f32,
    };

    pub fn init(config: Config) !Unit {
        var gpa = GPA{};
        var log = try Log.init(config.log);
        errdefer {
            if (gpa.deinit() == .leak) {
                log.err("Memory leaks detected", .{});
            }
        }

        const self_ptr = try gpa.allocator().create(Self);
        self_ptr.* = IMU{
            .log = log,
            .gpa = gpa,
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

        var log = self.log;
        var gpa = self.gpa;
        gpa.allocator().destroy(self);

        if (gpa.deinit() == .leak) {
            log.err("Memory leaks detected", .{});
        }
        log.deinit();
    }

    pub fn start(ctx: *anyopaque) void {
        _ = ctx;
    }

    pub fn stop(ctx: *anyopaque) void {
        _ = ctx;
    }

    fn update(ctx: *anyopaque) !void {
        _ = ctx;
    }
};
