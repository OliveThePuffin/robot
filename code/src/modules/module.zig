const std = @import("std");
const log = @import("logger.zig").log;
const Gateway = @import("gateway.zig").Gateway;

pub fn Module(comptime GatewayType: type, comptime TaskQueueType: type) type {
    return struct {
        const Self = @This();

        pub const TaskQueue = TaskQueueType;

        // Some things a module should do
        // - Start
        // - Stop
        // - Module specific requests
        // - Clear all requests

        name: []const u8 = "Unnamed Module",
        allocator: std.mem.Allocator,

        // Gateway maps enums to functions with args
        gateway: GatewayType,

        task_queue: TaskQueue = undefined,
        semaphore: std.Thread.Semaphore = .{ .permits = 0 },
        queue_mutex: std.Thread.Mutex = .{},
        consumer_thread: std.Thread = undefined,

        pub fn init(name: []const u8, gateway: GatewayType, task_queue: TaskQueue, allocator: std.mem.Allocator) !*Self {
            log(.INFO, name, "Initializing module", .{});
            const module = try allocator.create(Self);
            module.* = Self{
                .name = name,
                .allocator = allocator,
                .gateway = gateway,
                .task_queue = task_queue,
                .consumer_thread = try std.Thread.spawn(.{}, Self.loop, .{module}),
            };
            return module;
        }

        pub fn deinit(self: *Self) void {
            log(.INFO, self.name, "Deinitializing module", .{});
            self.task_queue.deinit();
            self.allocator.destroy(self);
            // TODO: kill consumer thread
        }

        /// Send a message
        pub fn send(self: *Self, topic: GatewayType.TopicEnum, message: GatewayType.MessageType) !void {
            const task = self.gateway.taskize(topic, message);
            self.queue_mutex.lock();
            try self.task_queue.add(task);
            self.queue_mutex.unlock();
            self.semaphore.post();
            log(.DEBUG, self.name, "Module send {} {any}", .{ topic, message });
        }

        fn loop(self: *Self) !void {
            log(.DEBUG, self.name, "Module {s} thread started", .{self.name});
            while (true) {
                self.semaphore.wait();
                self.queue_mutex.lock();
                const task = self.task_queue.remove();
                self.queue_mutex.unlock();
                log(.DEBUG, self.name, "Executing Job {any}", .{task});
                task.job(task.args);
            }
            log(.DEBUG, self.name, "Module {s} thread stopped", .{self.name});
        }
    };
}

test "module" {
    const Test = struct {
        const Self = @This();
        const TestGateway = Gateway(RequestEnum, MathStruct);
        const TestTaskQueue = std.PriorityQueue(TestGateway.Task, void, greaterThan);
        const TestModule = Module(TestGateway, TestTaskQueue);

        const RequestEnum = enum {
            ADD,
            SUB,
        };

        const MathStruct = struct {
            a: u32,
            b: u32,
            r: *u32,
        };

        fn greaterThan(context: void, a: TestGateway.Task, b: TestGateway.Task) std.math.Order {
            _ = context;
            return std.math.order(b.priority, a.priority);
        }

        fn add(math: MathStruct) void {
            math.r.* = math.a + math.b;
        }

        fn sub(math: MathStruct) void {
            math.r.* = math.a - math.b;
        }

        fn next_task(queue: TestModule.TaskQueue) usize {
            _ = queue;
            return 0;
        }

        pub fn init(name: []const u8) !*TestModule {
            var gateway = TestGateway{};
            gateway.registerMessageHandler(.ADD, Self.add, 1);
            gateway.registerMessageHandler(.SUB, Self.sub, 0);
            const allocator = std.heap.page_allocator;
            return try TestModule.init(name, gateway, TestTaskQueue.init(allocator, {}), allocator);
        }
    };

    var x: u32 = 0;
    var module_test = try Test.init("test");

    // Test sending
    std.time.sleep(0.1 * std.time.ns_per_s);
    try module_test.send(Test.RequestEnum.ADD, .{ .a = 5, .b = 3, .r = &x });
    std.time.sleep(0.1 * std.time.ns_per_s);
    try std.testing.expect(x == 8);
    std.time.sleep(0.1 * std.time.ns_per_s);
    try module_test.send(Test.RequestEnum.SUB, .{ .a = 5, .b = 3, .r = &x });
    std.time.sleep(0.1 * std.time.ns_per_s);
    try std.testing.expect(x == 2);
    std.time.sleep(0.1 * std.time.ns_per_s);

    // Deinitialize modules
    module_test.deinit();
}
