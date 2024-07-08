const std = @import("std");
const testing = std.testing;

export fn add(a: i32, b: i32) i32 {
    return a + b;
}

// TODO: add all unit tests. Apparently this doesn't work
test {
    std.testing.refAllDecls(@This());
}
