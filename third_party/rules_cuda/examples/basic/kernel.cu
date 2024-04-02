#include "kernel.h"

#include <iostream>

#define CUDA_CHECK(expr)                                                \
  do {                                                                  \
    cudaError_t err = (expr);                                           \
    if (err != cudaSuccess) {                                           \
      fprintf(stderr, "CUDA Errors Code  : %d\n     Errors String: %s\n", \
              err, cudaGetErrorString(err));                            \
      exit(err);                                                        \
    }                                                                   \
  } while (0)

__global__ void kernel() {
  printf("cuda kernel called!\n");
}

void launch() {
  kernel<<<1, 1>>>();
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
}
