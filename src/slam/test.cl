__kernel void matmul(__global float* A, __global float* B, __global float* C, int heightA, int widthA, int widthB) {
    int row = get_global_id(0);
    int col = get_global_id(1);

    if (row < heightA && col < widthB) {
        float sum = 0.0f;
        for (int k = 0; k < widthA; k++) {
            sum += A[row * widthA + k] * B[k * widthB + col];
        }
        C[row * widthB + col] = sum;
    }
}
