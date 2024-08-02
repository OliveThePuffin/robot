const std = @import("std");
const log = @import("logger").log;
pub const cl = @import("opencl");

const name = "OpenCL";

pub const OpenCL = struct {
    const Self = @This();
    aa: std.heap.ArenaAllocator,

    platform_id: cl.platform.cl_platform_id,
    devices: []cl.device.cl_device_id,
    device_id: cl.device.cl_device_id,
    context: cl.context.cl_context,
    queue: cl.command_queue.cl_command_queue,

    pub const OpenCLError = error{
        PlatformNotFound,
        DeviceNotFound,
    };

    pub const Config = struct {
        platform_name: []const u8,
        device_name: []const u8,
    };

    pub fn compileOpenclSrc(self: *Self, sources: []const []const u8) !cl.program.cl_program {
        const allocator = self.aa.allocator();
        log(.INFO, name, "Compiling Kernel...", .{});
        const program = try cl.program.create_with_source(self.context, sources, allocator);

        cl.program.build(program, &[_]cl.device.cl_device_id{self.device_id}, null, null, null) catch |err| {
            if (err == cl.errors.opencl_error.build_program_failure) {
                var build_log_size: usize = undefined;
                try cl.program.get_build_info(program, self.device_id, cl.program.enums.build_info.build_log, 0, null, &build_log_size);

                const build_log: []u8 = try allocator.alloc(u8, build_log_size);
                defer allocator.free(build_log);

                try cl.program.get_build_info(program, self.device_id, cl.program.enums.build_info.build_log, build_log_size, build_log.ptr, null);

                log(.ERROR, name, "Build log: {s}", .{build_log});
            }

            return err;
        };
        return program;
    }

    pub fn init(config: Config) !OpenCL {
        log(.INFO, name, "Initializing", .{});

        var aa = std.heap.ArenaAllocator.init(std.heap.page_allocator);
        const allocator = aa.allocator();
        const platform_id = try choosePlatform(config.platform_name, allocator);
        const devices = try getDevices(platform_id, allocator);
        const device_id = try chooseDevice(devices, config.device_name, allocator);
        const context = try cl.context.create(null, devices, null, null);
        const queue = try cl.command_queue.create(context, device_id, 0);
        return OpenCL{
            .aa = aa,
            .platform_id = platform_id,
            .devices = devices,
            .device_id = device_id,
            .context = context,
            .queue = queue,
        };
    }

    pub fn deinit(self: *Self) void {
        log(.INFO, name, "Deinitializing", .{});
        // All the OpenCL functions can fail. According to zig zen, resource deallocation must succeed.
        cl.context.release(self.context) catch unreachable;
        cl.command_queue.release(self.queue) catch unreachable;
        self.aa.deinit();
    }

    fn choosePlatform(platform_name_config: []const u8, allocator: std.mem.Allocator) !cl.platform.cl_platform_id {
        const platforms: []cl.platform.platform_info = try cl.platform.get_all(allocator);

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

    fn getDevices(platform_id: cl.platform.cl_platform_id, allocator: std.mem.Allocator) ![]cl.device.cl_device_id {
        var number_of_devices: u32 = undefined;
        try cl.device.get_ids(platform_id, cl.device.enums.device_type.all, null, &number_of_devices);
        const devices = try allocator.alloc(cl.device.cl_device_id, number_of_devices);
        try cl.device.get_ids(platform_id, cl.device.enums.device_type.all, devices, null);
        return devices;
    }

    fn chooseDevice(devices: []cl.device.cl_device_id, device_name_config: []const u8, allocator: std.mem.Allocator) !cl.device.cl_device_id {
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
};
test {}
