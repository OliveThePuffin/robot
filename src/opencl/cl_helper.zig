const std = @import("std");
const Log = @import("Log").Log;
pub const cl = @import("opencl");

const name = "OpenCL";

pub const OpenCLError = error{
    PlatformNotFound,
    DeviceNotFound,
};

pub const Config = struct {
    platform_name: []const u8,
    device_name: []const u8,
};

pub fn compileOpenclSrc(
    sources: []const []const u8,
    allocator: std.mem.Allocator,
    context: cl.context.cl_context,
    device_id: cl.device.cl_device_id,
    log: *Log,
) !cl.program.cl_program {
    log.info("Compiling OpenCL Program...", .{});
    const program = try cl.program.create_with_source(context, sources, allocator);

    cl.program.build(program, &[_]cl.device.cl_device_id{device_id}, null, null, null) catch |err| {
        if (err == cl.errors.opencl_error.build_program_failure) {
            var build_log_size: usize = undefined;
            try cl.program.get_build_info(program, device_id, cl.program.enums.build_info.build_log, 0, null, &build_log_size);

            const build_log: []u8 = try allocator.alloc(u8, build_log_size);
            defer allocator.free(build_log);

            try cl.program.get_build_info(program, device_id, cl.program.enums.build_info.build_log, build_log_size, build_log.ptr, null);

            log.err("Build log: {s}", .{build_log});
        }

        return err;
    };
    return program;
}

pub fn choosePlatform(platform_name_config: []const u8, allocator: std.mem.Allocator, log: *Log) !cl.platform.cl_platform_id {
    const platforms: []cl.platform.platform_info = try cl.platform.get_all(allocator);
    defer allocator.free(platforms);

    var chosen: ?usize = null;

    // List all platforms
    for (platforms, 0..) |platform, i| {
        const fields = @typeInfo(@TypeOf(platform)).Struct.fields;
        log.debug("Platform {}:", .{i});
        inline for (fields) |field| {
            if (field.type == cl.platform.cl_platform_id) {
                log.debug("{s} = {}", .{ field.name, @as(*u8, @ptrCast(@field(platform, field.name))).* });
            } else {
                log.debug(
                    "{s} = {s}",
                    .{ field.name, @as([*:0]u8, @ptrCast(@field(platform, field.name))) },
                );
            }
        }
        const platform_name_sentinel = @as([*:0]u8, @ptrCast(platform.name.ptr));
        if (std.mem.eql(u8, platform_name_config, std.mem.span(platform_name_sentinel))) {
            chosen = i;
        }
        log.debug("", .{});
    }

    if (chosen) |c| {
        const platform_name_sentinel = @as([*:0]u8, @ptrCast(platforms[c].name.ptr));
        log.info("Platform Chosen: {s}", .{platform_name_sentinel});
        return platforms[c].id;
    } else {
        log.err("Platform {s} not found", .{platform_name_config});
        return OpenCLError.PlatformNotFound;
    }
}

pub fn getDevices(platform_id: cl.platform.cl_platform_id, allocator: std.mem.Allocator) ![]cl.device.cl_device_id {
    var number_of_devices: u32 = undefined;
    try cl.device.get_ids(platform_id, cl.device.enums.device_type.all, null, &number_of_devices);
    const devices = try allocator.alloc(cl.device.cl_device_id, number_of_devices);
    try cl.device.get_ids(platform_id, cl.device.enums.device_type.all, devices, null);
    return devices;
}

pub fn chooseDevice(devices: []cl.device.cl_device_id, device_name_config: []const u8, allocator: std.mem.Allocator, log: *Log) !cl.device.cl_device_id {
    for (devices) |device| {
        log.debug("Device ID: {}", .{@as(*u8, @ptrCast(device.?)).*});

        // Device Type
        var device_type: u64 = undefined;
        try cl.device.get_info(device, cl.device.enums.device_info.type, @sizeOf(u64), @ptrCast(&device_type), null);
        log.debug("Device Type: {s}", .{@tagName(@as(cl.device.enums.device_type, @enumFromInt(device_type)))});

        // Device Name
        var device_name_size: usize = undefined;
        try cl.device.get_info(device, cl.device.enums.device_info.name, 0, null, &device_name_size);
        const device_name: []u8 = try allocator.alloc(u8, device_name_size);
        defer allocator.free(device_name);
        try cl.device.get_info(device, cl.device.enums.device_info.name, device_name_size, @ptrCast(device_name), null);
        const device_name_sentinel = @as([*:0]u8, @ptrCast(device_name.ptr));
        log.debug("Device Name: {s}", .{device_name_sentinel});

        if (std.mem.eql(u8, device_name_config, std.mem.span(device_name_sentinel))) {
            log.info("Device Chosen: {s}", .{device_name_sentinel});
            return device;
        }
    }

    log.err("Platform {s} not found", .{device_name_config});
    return OpenCLError.DeviceNotFound;
}
