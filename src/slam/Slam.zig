const std = @import("std");
const log = @import("logger").log;
const rs_depth = @import("rs-depth.zig");
const OpenCL = @import("OpenCL").OpenCL;
const cl = @import("OpenCL").cl;
const Slam = @This();

const name = "SLAM";
var gpa = std.heap.GeneralPurposeAllocator(.{}){};
var allocator = gpa.allocator();
var opencl: OpenCL = undefined;

var kernel_matmul: cl.kernel.cl_kernel = undefined;
var program_matmul: cl.program.cl_program = undefined;
var a_buffer: cl.buffer.cl_mem = undefined;
var b_buffer: cl.buffer.cl_mem = undefined;
var c_buffer: cl.buffer.cl_mem = undefined;

const matmul_src = @embedFile("test.cl");

pub const Config = struct {
    opencl: OpenCL.Config,
    dry_run: bool,
};

const KernelBufferPair = struct {
    kernel: cl.program.cl_program,
    buffer: []cl.buffer.cl_mem,
};

pub fn start() void {
    log(.INFO, name, "Starting", .{});
    rs_depth.start_loop();
}

pub fn stop() void {
    log(.INFO, name, "Stopping", .{});
    rs_depth.stop_loop();
}

pub fn init(config: Config) !void {
    log(.INFO, name, "Initializing", .{});

    opencl = try OpenCL.init(config.opencl);

    program_matmul = try opencl.compileOpenclSrc(&[_][]const u8{matmul_src});

    try setupBuffers();

    // ...

    kernel_matmul = try cl.kernel.create(program_matmul, "matmul");
    defer {}

    const rows_a: u32 = 4;
    const cols_a: u32 = 3;
    const rows_b = cols_a;
    const cols_b: u32 = 4;
    const rows_c = rows_a;
    const cols_c = cols_b;

    try cl.kernel.set_arg(kernel_matmul, 0, @sizeOf(cl.buffer.cl_mem), @ptrCast(&a_buffer));
    try cl.kernel.set_arg(kernel_matmul, 1, @sizeOf(cl.buffer.cl_mem), @ptrCast(&b_buffer));
    try cl.kernel.set_arg(kernel_matmul, 2, @sizeOf(cl.buffer.cl_mem), @ptrCast(&c_buffer));
    try cl.kernel.set_arg(kernel_matmul, 3, @sizeOf(u32), &rows_a);
    try cl.kernel.set_arg(kernel_matmul, 4, @sizeOf(u32), &cols_a);
    try cl.kernel.set_arg(kernel_matmul, 5, @sizeOf(u32), &cols_b);

    var event: cl.event.cl_event = undefined;
    try cl.kernel.enqueue_nd_range(opencl.queue, kernel_matmul, null, &[2]usize{ rows_a, cols_b }, null, null, &event);
    try cl.event.wait(event);
    try cl.event.release(event);

    var a: [rows_a * cols_a]f32 = undefined;
    var b: [rows_b * cols_b]f32 = undefined;
    var c: [rows_c * cols_c]f32 = undefined;
    try cl.buffer.read(opencl.queue, a_buffer, true, 0, rows_a * cols_a * @sizeOf(f32), &a, null, null);
    try cl.buffer.read(opencl.queue, b_buffer, true, 0, rows_b * cols_b * @sizeOf(f32), &b, null, null);
    try cl.buffer.read(opencl.queue, c_buffer, true, 0, rows_c * cols_c * @sizeOf(f32), &c, null, null);

    log(.INFO, name, "Buffer A: {d}", .{a});
    log(.INFO, name, "Buffer B: {d}", .{b});
    log(.INFO, name, "Buffer C: {d}", .{c});

    rs_depth.module_config = .{ .dry_run = config.dry_run };
}

pub fn deinit() void {
    log(.INFO, name, "Deinitializing", .{});

    cl.kernel.release(kernel_matmul) catch unreachable;
    cl.program.release(program_matmul) catch unreachable;
    cl.buffer.release(a_buffer) catch unreachable;
    cl.buffer.release(b_buffer) catch unreachable;
    cl.buffer.release(c_buffer) catch unreachable;

    opencl.deinit();

    if (gpa.deinit() == .leak) {
        log(.ERROR, name, "Memory leaks detected", .{});
    }
}

// Setup OpenCL buffers
fn setupBuffers() !void {
    const rows_a = 4;
    const cols_a = 3;
    const rows_b = cols_a;
    const cols_b = 4;
    const rows_c = rows_a;
    const cols_c = cols_b;

    const a = [rows_a * cols_a]f32{
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
        0, 0, 0,
    };
    const b = [rows_b * cols_b]f32{
        1, 2, 3, 4,
        2, 1, 2, 3,
        3, 2, 1, 2,
    };

    a_buffer = try cl.buffer.create(
        opencl.context,
        @intFromEnum(cl.buffer.enums.mem_flags.read_only),
        rows_a * cols_a * @sizeOf(f32),
        null,
    );
    b_buffer = try cl.buffer.create(
        opencl.context,
        @intFromEnum(cl.buffer.enums.mem_flags.read_only),
        rows_b * cols_b * @sizeOf(f32),
        null,
    );
    c_buffer = try cl.buffer.create(
        opencl.context,
        @intFromEnum(cl.buffer.enums.mem_flags.write_only),
        rows_c * cols_c * @sizeOf(f32),
        null,
    );
    try cl.buffer.write(opencl.queue, a_buffer, true, 0, rows_a * cols_a * @sizeOf(f32), &a, null, null);
    try cl.buffer.write(opencl.queue, b_buffer, true, 0, rows_b * cols_b * @sizeOf(f32), &b, null, null);
}

// TODO: also try adding a kalman filter here

test {}
