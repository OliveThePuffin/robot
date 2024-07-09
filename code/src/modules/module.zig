const std = @import("std");
const log = @import("../utils/logger.zig").log;

pub fn Module(comptime GatewayType: type) type {
    return struct {
        const Self = @This();
        const Scheduler = *const fn (TaskQueue) usize;

        fn greaterThan(context: void, a: GatewayType.Task, b: GatewayType.Task) std.math.Order {
            _ = context;
            return std.math.order(b, a);
        }
        pub const TaskQueue = std.PriorityQueue(GatewayType.Task, void, greaterThan);

        // Some things a module should do
        // - Start
        // - Stop
        // - Module specific requests
        // - Clear all requests
        name: []const u8 = "Unnamed Module",

        // Gateway maps enums to functions with args
        gateway: GatewayType,

        // Scheduler decides which task to run next in the task queue
        scheduler: Scheduler,
        task_queue: TaskQueue,

        // The ModuleType module is meant to dictate the behavior of the Module module
        // It returns/holds the request enums, the Gateway handler table, and the scheduler function
        initialized: bool = false,

        pub fn init(name: []const u8, gateway: GatewayType, scheduler: Scheduler) Self {
            log(.INFO, name, "Initializing module", .{});
            return .{
                .name = name,
                .initialized = true,
                .gateway = gateway,
                .scheduler = scheduler,
                .task_queue = undefined,
            };
        }

        pub fn deinit(self: *Self) void {
            if (!self.initialized) {
                log(.WARN, self.name, "Cannot deinitialize module: not initialized", .{});
                return;
            }
            log(.INFO, self.name, "Deinitializing module", .{});
            self.initialized = false;
        }

        /// Send a message
        pub fn send(self: *Self, topic: self.gateway.TopicEnum, message: self.gateway.MessageType) !void {
            const task = try self.gateway.taskize(topic, message);
            self.task_queue.add(task);
        }
    };
}

// TODO: implement and test gateway, scheduler, and queue
// TODO: spawn process thread on init that does tasks in the queue until deinit. test that too
test "scheduler" {
    try std.testing.expect(true);
}
