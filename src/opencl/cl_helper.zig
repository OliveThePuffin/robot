const std = @import("std");
const log = @import("logger").log;
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
) !cl.program.cl_program {
    log(.INFO, name, "Compiling OpenCL Program...", .{});
    const program = try cl.program.create_with_source(context, sources, allocator);

    cl.program.build(program, &[_]cl.device.cl_device_id{device_id}, null, null, null) catch |err| {
        if (err == cl.errors.opencl_error.build_program_failure) {
            var build_log_size: usize = undefined;
            try cl.program.get_build_info(program, device_id, cl.program.enums.build_info.build_log, 0, null, &build_log_size);

            const build_log: []u8 = try allocator.alloc(u8, build_log_size);
            defer allocator.free(build_log);

            try cl.program.get_build_info(program, device_id, cl.program.enums.build_info.build_log, build_log_size, build_log.ptr, null);

            log(.ERROR, name, "Build log: {s}", .{build_log});
        }

        return err;
    };
    return program;
}

pub fn choosePlatform(platform_name_config: []const u8, allocator: std.mem.Allocator) !cl.platform.cl_platform_id {
    const platforms: []cl.platform.platform_info = try cl.platform.get_all(allocator);
    defer allocator.free(platforms);

    var chosen: ?usize = null;

    // List all platforms
    for (platforms, 0..) |platform, i| {
        const fields = @typeInfo(@TypeOf(platform)).Struct.fields;
        log(.DEBUG, name, "Platform {}:", .{i});
        inline for (fields) |field| {
            if (field.type == cl.platform.cl_platform_id) {
                log(.DEBUG, name, "{s} = {}", .{ field.name, @as(*u8, @ptrCast(@field(platform, field.name))).* });
            } else {
                log(.DEBUG, name, "{s} = {s}", .{ field.name, @field(platform, field.name) });
            }
        }
        if (std.mem.eql(u8, platform_name_config, platform.name[0 .. platform.name.len - 1])) {
            chosen = i;
        }
        log(.DEBUG, name, "", .{});
    }

    if (chosen) |c| {
        log(.INFO, name, "Platform Chosen: {s}", .{platforms[c].name});
        return platforms[c].id;
    } else {
        log(.ERROR, name, "Platform {s} not found", .{platform_name_config});
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

pub fn chooseDevice(devices: []cl.device.cl_device_id, device_name_config: []const u8, allocator: std.mem.Allocator) !cl.device.cl_device_id {
    var chosen_device_id: cl.device.cl_device_id = null;
    var chosen_device_name: ?[]u8 = null;

    for (devices) |device| {
        log(.DEBUG, name, "Device ID: {}", .{@as(*u8, @ptrCast(device.?)).*});

        // Device Type
        var device_type: u64 = undefined;
        try cl.device.get_info(device, cl.device.enums.device_info.type, @sizeOf(u64), @ptrCast(&device_type), null);
        log(.DEBUG, name, "Device Type: {s}", .{@tagName(@as(cl.device.enums.device_type, @enumFromInt(device_type)))});

        // Device Name
        var device_name_size: usize = undefined;
        try cl.device.get_info(device, cl.device.enums.device_info.name, 0, null, &device_name_size);
        const device_name: []u8 = try allocator.alloc(u8, device_name_size);
        defer allocator.free(device_name);
        try cl.device.get_info(device, cl.device.enums.device_info.name, device_name_size, @ptrCast(device_name), null);
        log(.DEBUG, name, "Device Name: {s}", .{device_name});

        if (std.mem.eql(u8, device_name_config, device_name[0 .. device_name.len - 1])) {
            chosen_device_id = device;
            chosen_device_name = device_name;
        }
    }

    if (chosen_device_id) |c| {
        log(.INFO, name, "Device Chosen: {s}", .{chosen_device_name.?});
        return c;
    } else {
        log(.ERROR, name, "Platform {s} not found", .{device_name_config});
        return OpenCLError.DeviceNotFound;
    }
}
