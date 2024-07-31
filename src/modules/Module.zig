const std = @import("std");
const log = @import("logger.zig").log;
//const Gateway = @import("gateway.zig").Gateway;
const Module = @This();

const TaskQueue = *anyopaque;
const Gateway = *anyopaque;
const Task = *const anyopaque;

name: []const u8 = "Unnamed Module",
core: Core,
allocator: std.mem.Allocator,
thread_state: ThreadInfo = .{},

const Core = struct {
    task_queue: ?TaskQueue,
    gateway: Gateway,

    add: *const fn (TaskQueue, Task) anyerror!void,
    pop: *const fn (TaskQueue) Task,
    execute: *const fn (std.mem.Allocator, Task) void,
    taskize: *const fn (std.mem.Allocator, Gateway, *const anyopaque, *anyopaque) anyerror!Task,
    deinit: *const fn (*Module) void,
};

const ThreadInfo = struct {
    semaphore: std.Thread.Semaphore = .{ .permits = 0 },
    queue_mutex: std.Thread.Mutex = .{},
    consumer_thread: std.Thread = undefined,
};

pub fn send(self: *Module, topic: anytype, message: anytype) !void {
    const opaque_topic: *const anyopaque = @ptrCast(&topic);
    const opaque_message: *anyopaque = @constCast(@ptrCast(message));
    self.thread_state.queue_mutex.lock();
    if (self.core.task_queue) |task_queue| {
        const task = try self.core.taskize(self.allocator, self.core.gateway, opaque_topic, opaque_message);
        try self.core.add(task_queue, task);
        self.thread_state.queue_mutex.unlock();
        self.thread_state.semaphore.post();
        log(.DEBUG, self.name, "Send\n\tTopic: {}\n\tMsg: {any}", .{ topic, message });
    } else {
        self.thread_state.queue_mutex.unlock();
        log(.WARN, self.name, "No task queue. For synchronous send, use sendSync", .{});
        return;
    }
}

fn consumeLoop(self: *Module) !void {
    log(.DEBUG, self.name, "Consumer loop started", .{});
    while (self.core.task_queue) |task_queue| {
        self.thread_state.semaphore.wait();
        self.thread_state.queue_mutex.lock();
        const task: Task = self.core.pop(task_queue);
        self.thread_state.queue_mutex.unlock();
        self.core.execute(self.allocator, task);
    }
    log(.DEBUG, self.name, "Consumer loop stopped", .{});
}

pub fn startConsumer(self: *Module) !void {
    self.thread_state.consumer_thread = try std.Thread.spawn(.{}, consumeLoop, .{self});
}

pub fn init(name: []const u8, allocator: std.mem.Allocator, core: Core) !*Module {
    const module = try allocator.create(Module);
    module.* = Module{
        .name = name,
        .allocator = allocator,
        .core = core,
    };
    try module.startConsumer();
    return module;
}

pub fn deinit(self: *Module) void {
    log(.INFO, self.name, "Deinitializing...", .{});
    self.thread_state.consumer_thread.detach();
    self.thread_state.queue_mutex.lock();
    self.core.deinit(self);
    self.thread_state.queue_mutex.unlock();
    self.allocator.destroy(self);
}

test "module" {
    const logLevelSet = @import("logger.zig").logLevelSet;
    logLevelSet(.NONE);
    var aa = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    const Test = struct {
        const Self = @This();
        const GatewayT = @import("gateway.zig").Gateway;
        const TestGateway = GatewayT(RequestEnum, MathStruct);
        const TestTaskQueue = std.PriorityQueue(*const TestGateway.Task, void, greaterThan);

        const RequestEnum = enum {
            ADD,
            SUB,
        };

        const MathStruct = struct {
            a: u32,
            b: u32,
            r: *u32,
        };

        fn greaterThan(context: void, a: *const TestGateway.Task, b: *const TestGateway.Task) std.math.Order {
            _ = context;
            return std.math.order(b.priority, a.priority);
        }

        fn add(math: MathStruct) void {
            math.r.* = math.a + math.b;
        }

        fn sub(math: MathStruct) void {
            math.r.* = math.a - math.b;
        }
        fn addTaskOpaque(task_queue: *anyopaque, task: *const anyopaque) !void {
            const tq: *TestTaskQueue = @ptrCast(@alignCast(task_queue));
            const t: *const TestGateway.Task = @ptrCast(@alignCast(task));
            try TestTaskQueue.add(tq, t);
        }

        fn popTaskOpaque(task_queue: *anyopaque) *const anyopaque {
            const tq: *TestTaskQueue = @ptrCast(@alignCast(task_queue));
            const t = tq.remove();
            return @as(*const anyopaque, @ptrCast(t));
        }

        fn executeOpaque(allocator: std.mem.Allocator, task: *const anyopaque) void {
            const t: *const TestGateway.Task = @ptrCast(@alignCast(task));
            TestGateway.execute(t.*);
            allocator.destroy(t);
        }

        fn taskizeOpaque(a: std.mem.Allocator, gateway: *anyopaque, topic: *const anyopaque, message: *anyopaque) !*anyopaque {
            const gw: *TestGateway = @ptrCast(@alignCast(gateway));
            const tc: *const TestGateway.TopicEnum = @ptrCast(@alignCast(topic));
            const m: *TestGateway.MessageType = @ptrCast(@alignCast(message));
            const t = try a.create(TestGateway.Task);
            t.* = TestGateway.taskize(gw, tc.*, m.*);
            return t;
        }

        fn deinit(self: *Module) void {
            const gateway: *TestGateway = @ptrCast(@alignCast(self.core.gateway));
            const task_queue: *TestTaskQueue = @ptrCast(@alignCast(self.core.task_queue));
            self.allocator.destroy(gateway);
            task_queue.deinit();
            self.allocator.destroy(task_queue);
        }

        pub fn init(name: []const u8, a: std.mem.Allocator) !*Module {
            const gateway = try a.create(TestGateway);
            gateway.* = TestGateway{};
            gateway.registerMessageHandler(.ADD, Self.add, 1);
            gateway.registerMessageHandler(.SUB, Self.sub, 0);

            const task_queue = try a.create(TestTaskQueue);
            task_queue.* = TestTaskQueue.init(std.testing.allocator, {});

            return try Module.init(name, a, .{
                .task_queue = task_queue,
                .gateway = gateway,

                .add = addTaskOpaque,
                .pop = popTaskOpaque,
                .execute = executeOpaque,
                .taskize = taskizeOpaque,
                .deinit = Self.deinit,
            });
        }
        var x: u32 = 0;
    };

    var module_test = try Test.init("test", aa.allocator());

    // Test sending
    std.time.sleep(0.1 * std.time.ns_per_s);
    try module_test.send(Test.RequestEnum.ADD, &(Test.MathStruct{ .a = 5, .b = 3, .r = &Test.x }));
    std.time.sleep(0.1 * std.time.ns_per_s);
    try std.testing.expect(Test.x == 8);
    std.time.sleep(0.1 * std.time.ns_per_s);
    try module_test.send(Test.RequestEnum.SUB, &(Test.MathStruct{ .a = 5, .b = 3, .r = &Test.x }));
    std.time.sleep(0.1 * std.time.ns_per_s);
    try std.testing.expect(Test.x == 2);
    std.time.sleep(0.1 * std.time.ns_per_s);

    // Deinitialize modules
    module_test.deinit();
}
