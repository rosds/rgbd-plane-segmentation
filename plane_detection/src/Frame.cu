ushort *d_in;
float *d_out;

__global__ void _cudaReadPointCloud(
        ushort *d_in, 
        float *d_out, 
        const int width, 
        const int height
) {
    unsigned u = blockDim.x * blockIdx.x + threadIdx.x;
    unsigned v = blockDim.y * blockIdx.y + threadIdx.y;
    unsigned idx = width * v + u;

    float depth = (float)d_in[idx] / 5000.0f;
    d_out[idx * 4] = (u - 318.6f) * depth / 517.3f;  
    d_out[idx * 4 + 1] = (v - 255.3f) * depth / 516.5f;
    d_out[idx * 4 + 2] = depth;
    d_out[idx * 4 + 3] = 1.0f;
} 

extern void cudaReadPointCloud(
        ushort *h_in, 
        float *h_out,
        const int width, 
        const int height
) {
   
    dim3 blockSize = dim3(width / 40, height / 40);
    dim3 gridSize = dim3(40, 40);

    cudaSetDevice(0);

    cudaMalloc(&d_in, width * height * sizeof(ushort));
    cudaMalloc(&d_out, 4 * width * height * sizeof(float));

    cudaMemcpy(d_in, h_in, width * height * sizeof(ushort), cudaMemcpyHostToDevice);
    _cudaReadPointCloud <<<gridSize, blockSize>>> (d_in, d_out, width, height);
    cudaMemcpy(h_out, d_out, 4 * width * height * sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(d_in);
    cudaFree(d_out);
}
