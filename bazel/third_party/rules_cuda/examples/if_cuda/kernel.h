#include <cstdio>

#define CUDA_CHECK(expr)                                                \
  do {                                                                  \
    cudaError_t err = (expr);                                           \
    if (err != cudaSuccess) {                                           \
      fprintf(stderr, "CUDA Errors Code  : %d\n     Errors String: %s\n", \
              err, cudaGetErrorString(err));                            \
      exit(err);                                                        \
    }                                                                   \
  } while (0)

void launch();
