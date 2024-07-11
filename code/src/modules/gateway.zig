const std = @import("std");

/// Gateway handles messages between modules.
pub fn Gateway(comptime TopicEnumCT: type, comptime MessageTypeCT: type) type {
    std.debug.assert(@typeInfo(TopicEnumCT) == .Enum);
    return struct {
        const Self = @This();
        const Handler = *const fn (MessageType) void;
        pub const TopicEnum = TopicEnumCT;
        pub const MessageType = MessageTypeCT;
        const TopicCount = @typeInfo(TopicEnum).Enum.fields.len;
        pub const Task = struct {
            job: Handler,
            args: MessageType,
            priority: usize,
        };

        handler_list: [TopicCount]Handler = undefined,
        priority_list: [TopicCount]usize = undefined,

        /// Returns a task list that must be executed and freed
        pub fn taskize(self: *Self, topic: TopicEnum, message: MessageType) Task {
            return Task{
                .job = self.handler_list[@intFromEnum(topic)],
                .args = message,
                .priority = self.priority_list[@intFromEnum(topic)],
            };
        }

        /// Register a message handler to one or more topics
        /// Overwrites any previously registered handler.
        pub fn registerMessageHandler(self: *Self, topic: TopicEnum, handler: Handler, priority: usize) void {
            self.handler_list[@intFromEnum(topic)] = handler;
            self.priority_list[@intFromEnum(topic)] = priority;
        }
    };
}

test "gateway" {
    const TestFunctions = struct {
        const Self = @This();
        var val: u32 = 0;

        pub fn handleMessage1(message: u32) void {
            Self.val += message * 1;
        }

        pub fn handleMessage2(message: u32) void {
            Self.val += message * 2;
        }

        pub fn handleMessage3(message: u32) void {
            Self.val += message * 3;
        }
    };
    const Topics = enum(u2) {
        topic1,
        topic2,
        topic3,
    };

    var gateway = Gateway(Topics, u32){};

    gateway.registerMessageHandler(Topics.topic1, TestFunctions.handleMessage1, 0);
    gateway.registerMessageHandler(Topics.topic2, TestFunctions.handleMessage2, 1);
    gateway.registerMessageHandler(Topics.topic3, TestFunctions.handleMessage2, 2);

    var result = gateway.taskize(Topics.topic1, 1);
    try std.testing.expect(result.job == TestFunctions.handleMessage1 and result.args == 1 and result.priority == 0);
    result = gateway.taskize(Topics.topic2, 2);
    try std.testing.expect(result.job == TestFunctions.handleMessage2 and result.args == 2 and result.priority == 1);
    result = gateway.taskize(Topics.topic3, 3);
    try std.testing.expect(result.job == TestFunctions.handleMessage2 and result.args == 3 and result.priority == 2);

    gateway.registerMessageHandler(Topics.topic3, TestFunctions.handleMessage3, 3);

    result = gateway.taskize(Topics.topic3, 4);
    try std.testing.expect(result.job == TestFunctions.handleMessage3 and result.args == 4 and result.priority == 3);
}
