const std = @import("std");
const log = @import("../logger.zig").log;
const rs_depth = @import("rs-depth.zig");
const Module = @import("../Module.zig");
const Gateway = @import("../gateway.zig").Gateway;

const Slam = @This();
pub const SlamGateway = Gateway(Request, void);
pub const SlamTaskQueue = std.PriorityQueue(*const SlamGateway.Task, void, greaterThan);

pub const Request = enum {
    START,
    STOP,
};

pub const Config = struct {
    dry_run: bool,
};

fn start(_: void) void {
    log(.INFO, "Slam", "Starting", .{});
    rs_depth.start_loop();
}

fn stop(_: void) void {
    log(.INFO, "Slam", "Stopping", .{});
    rs_depth.stop_loop();
}

fn greaterThan(context: void, a: *const SlamGateway.Task, b: *const SlamGateway.Task) std.math.Order {
    _ = context;
    return std.math.order(b.priority, a.priority);
}

pub fn init(name: []const u8, allocator: std.mem.Allocator, config: Config) ?*Module {
    rs_depth.module_config = .{ .dry_run = config.dry_run };
    const gateway = allocator.create(SlamGateway) catch |e| {
        log(.ERROR, name, "Could not create the slam gateway: {}", .{e});
        return null;
    };
    gateway.* = SlamGateway{};
    gateway.registerMessageHandler(.START, Slam.start, 1);
    gateway.registerMessageHandler(.STOP, Slam.stop, 0);
    const task_queue = allocator.create(SlamTaskQueue) catch |e| {
        log(.ERROR, name, "Could not create the slam task queue: {}", .{e});
        return null;
    };
    task_queue.* = SlamTaskQueue.init(allocator, {});

    return Module.init(name, allocator, .{
        .task_queue = task_queue,
        .gateway = gateway,

        .add = addTaskOpaque,
        .pop = popTaskOpaque,
        .execute = executeOpaque,
        .taskize = taskizeOpaque,
        .deinit = deinit,
    }) catch |e| {
        log(.ERROR, name, "Could not create the Slam module: {}", .{e});
        return null;
    };
}

fn addTaskOpaque(task_queue: *anyopaque, task: *const anyopaque) !void {
    const tq: *SlamTaskQueue = @ptrCast(@alignCast(task_queue));
    const t: *const SlamGateway.Task = @ptrCast(@alignCast(task));
    try SlamTaskQueue.add(tq, t);
}

fn popTaskOpaque(task_queue: *anyopaque) *const anyopaque {
    const tq: *SlamTaskQueue = @ptrCast(@alignCast(task_queue));
    const t = tq.remove();
    return @as(*const anyopaque, @ptrCast(t));
}

fn executeOpaque(allocator: std.mem.Allocator, task: *const anyopaque) void {
    const t: *const SlamGateway.Task = @ptrCast(@alignCast(task));
    SlamGateway.execute(t.*);
    allocator.destroy(t);
}

fn taskizeOpaque(allocator: std.mem.Allocator, gateway: *anyopaque, topic: *const anyopaque, message: *anyopaque) !*anyopaque {
    const gw: *SlamGateway = @ptrCast(@alignCast(gateway));
    const tc: *const SlamGateway.TopicEnum = @ptrCast(@alignCast(topic));
    const m: *SlamGateway.MessageType = @ptrCast(@alignCast(message));
    const t = try allocator.create(SlamGateway.Task);
    t.* = SlamGateway.taskize(gw, tc.*, m.*);
    return t;
}

fn deinit(self: *Module) void {
    const gateway: *SlamGateway = @ptrCast(@alignCast(self.core.gateway));
    const task_queue: *SlamTaskQueue = @ptrCast(@alignCast(self.core.task_queue));
    self.allocator.destroy(gateway);
    task_queue.deinit();
    self.allocator.destroy(task_queue);
}

// TODO: also try adding a kalman filter here
