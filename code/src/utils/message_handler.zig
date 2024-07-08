const std = @import("std");

pub fn TopicHandlers(comptime TopicEnum: type, comptime MessageType: type, comptime TopicMaxHandlers: usize) type {
    std.debug.assert(@typeInfo(TopicEnum) == .Enum);
    return struct {
        const Self = @This();
        const Handler = *const fn (MessageType) void;
        const TopicCount = @typeInfo(TopicEnum).Enum.fields.len;

        comptime handler_lists: [TopicCount][TopicMaxHandlers]Handler = undefined,
        comptime handler_list_len: [TopicCount]usize = [_]usize{0} ** TopicCount,

        /// Register a message handler to one or more topics
        pub fn registerMessageHandler(self: *Self, comptime topics: []const TopicEnum, comptime handler: Handler) void {
            inline for (topics) |topic| {
                self.handler_lists[@intFromEnum(topic)][self.handler_list_len[@intFromEnum(topic)]] = handler;
                self.handler_list_len[@intFromEnum(topic)] += 1;
                // for (0..TopicCount) |i| {
                //     //@compileLog(@as(TopicEnum, @enumFromInt(i)), i, self.handler_lists[@intFromEnum(topic)].len);
                //     //@compileLog(self.handler_lists[@intFromEnum(topic)].handler_list);
                //     const list_data_print = self.handler_lists[i];
                //     @compileLog(@as(TopicEnum, @enumFromInt(i)), i, self.handler_list_len[i]);
                //     @compileLog(list_data_print);
                // }
            }
        }
    };
}
/// Send a message
pub fn send(topicHandlers: anytype, topic: anytype, message: anytype) !void {
    const handler_list = topicHandlers.handler_lists[@intFromEnum(topic)];
    for (0..topicHandlers.handler_list_len[@intFromEnum(topic)]) |i| {
        const thread = try std.Thread.spawn(.{}, handler_list[i], .{message});
        std.Thread.detach(thread);
        //handler_list[i](message);
    }
}

test "message handler" {
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

    comptime var topicHandlers = TopicHandlers(Topics, u32, 2){};
    comptime {
        topicHandlers.registerMessageHandler(&[1]Topics{Topics.topic1}, TestFunctions.handleMessage1);
        topicHandlers.registerMessageHandler(&[2]Topics{ Topics.topic2, Topics.topic3 }, TestFunctions.handleMessage2);
        topicHandlers.registerMessageHandler(&[1]Topics{Topics.topic3}, TestFunctions.handleMessage3);
    }

    try send(topicHandlers, Topics.topic1, 1);
    try std.testing.expect(TestFunctions.val == 1);
    try send(topicHandlers, Topics.topic2, 2);
    try std.testing.expect(TestFunctions.val == 5);
    try send(topicHandlers, Topics.topic2, 3);
    try std.testing.expect(TestFunctions.val == 11);
    try send(topicHandlers, Topics.topic3, 4);
    try std.testing.expect(TestFunctions.val == 31);
}
