# LinMatrix library
- LinMatrix is a lightweight matrix computation software library aim for MCU or embedded system.
- Pure C language library.
- No dynamic memory, no recursion, no global variable.
- Unit tested on X86-64/AMD64 platform.
- Although most of the functions have been unit tested, but considering the floating point computation error and any unexpected behaviors on different platform, unit testing still does not guarantee that any matrix calculation results will be all correct without any errors. It is necessary to carefully verify the calculation results by yourself according to your use requirement and scenario. **Use at your own risk**.
- Any feedback and suggestions are welcome.


# Features
- Support single or double precision (according to compilation settings).
- Handle-based matrix management and operation functions:
    - Easy to setup, clear and dump the matrix content of specific handle.
    - Allow to assign unique name to matrix handle.
- Support vector and matrix operations including:
    - zeros, copy, swap, permute, identity.
    - scalar, axpy(add, sub), gemm(multiplication).
	- abs, max, min, max_abs, min_abs.
    - bandwidth.
	- givens.
    - trace.
- Support advanced matrix computation functions including:
    - LU decomposition (including rank, determinant and inverse of matrix computation).
    - QR decomposition (based on householder reflection method).
    - Hessenberg similarity transform functions for **symmetric** matrix.
    - Finding the eigenvalues and eigenvectors of **symmetric** matrix.
        - Finding the square root of **symmetric** matrix.


# Limitations
- Not able to support both single and double precision computation at the same time (decided by [compile option](./inc/lm_global.h)).
- Support real number only (complex number is not supported).
- The maximum acceptable matrix size is 4096 by 4096.
- Using the lm_lu_rank function to check the rank of given LU matrix is unreliable.
- Using determinant (lm_lu_det) to check if a matrix is invertible (non-singular) is unreliable.
- There are only a few null pointer and input value checks in this library, please do not input invalid memory addresses to APIs.
- The matrix handle and its corresponding matrix data is not thread-safe, please do not access the same matrix handle/data from different threads at the same time.


# Build everything

```make 
make clean; make
```


# Run examples
```console
$ ./lm_mat_mult (or ./lm_mat_mult.exe on Windows)

Matrix A:
LM = [
         3.3333334e-01   6.6666669e-01  -6.6666669e-01;
        -6.6666669e-01   6.6666669e-01   3.3333334e-01;
         6.6666669e-01   3.3333334e-01   6.6666669e-01;
]


Matrix B:
LM = [
         3.3333334e-01  -6.6666669e-01   6.6666669e-01;
         6.6666669e-01   6.6666669e-01   3.3333334e-01;
        -6.6666669e-01   3.3333334e-01   6.6666669e-01;
]


Matrix A * B:
LM = [
         1.0000000e+00   0.0000000e+00   0.0000000e+00;
         0.0000000e+00   1.0000001e+00   0.0000000e+00;
         0.0000000e+00   0.0000000e+00   1.0000000e+00;
]
```


# Run unit test

```console
$ ./lm_ut
********** Testing Suite << lm_ut_mat_suites >> **********
[001] [Case lm_ut_mat_set_and_clr] ...
[002] [Case lm_ut_mat_elem_set_and_clr] ...

Cases (success/failure): 2 / 0
Suites (success/failure): 1 / 0
Failure suite list:
...
```


# Examples
### Please check the files in ["examples"](./examples/) folder for more details
```c
    result = lm_mat_set(&mat_a, ...);

    result = lm_mat_set(&mat_b, ...);

    result = lm_mat_set(&mat_c, ...);

    /* Compute C := A * B */
    result = lm_oper_gemm(false, false,
                          LM_MAT_ONE_VAL, &mat_a, &mat_b,
                          LM_MAT_ZERO_VAL, &mat_c);

    printf("\nMatrix A:\n");
    lm_mat_dump(&mat_a);

    printf("\nMatrix B:\n");
    lm_mat_dump(&mat_b);

    printf("\nMatrix A * B:\n");
    lm_mat_dump(&mat_c);
```


# Performance
### Test method: ["examples/lm_perf_measure/lm_perf_measure.c"](examples/lm_perf_measure/lm_perf_measure.c)
```
GEMM performance test
    Repeat 10 times:
        call GEMM to compute C := A  * B
        call GEMM to compute C := A' * B
        call GEMM to compute C := A  * B'
        call GEMM to compute C := A' * B'
        
    Repeat 10 times:
        call AXPY to compute A := 0.1 * C + A
        call AXPY to compute A := 2.0 * C + A
        call AXPY to compute B := 0.1 * A + B
        call AXPY to compute B := 2.0 * A + B
        call AXPY to compute C := 0.1 * B + C
        call AXPY to compute C := 2.0 * B + C
        
    The dimension of matrix A, B, C is 32 * 32.
```

#### (1) Result (**currently used**, unrolled loop version GEMM and AXPY):
Platform                                                                                              |     GEMM     |     AXPY     | Code size in lm_lib.a |    Note 
------------------------------------------------------------------------------------------------------|:------------:|-------------:| ---------------------:| ---------------------
PC, AMD Ryzen 9 5900HS, 3.30 GHz, 8 cores, 32 GB ram, VirtualBox & Linux 64, gcc v9.3.0               | 0.000632 sec | 0.000025 sec | 53864 bytes totals    | -O2, Single precision
Raspberry Pi 3, ARM Cortex-A53, ARMv7 (v7l), 1.2 GHz, 4 cores, 1 GB ram, Raspbian, gcc v6.3.0         | 0.026192 sec | 0.001490 sec | 42651 bytes totals    | -O2, Single precision
STM32F411RE, Arm Cortex-M4 core, 100 MHz, 128 KB SRAM, **HW FPU**, No OS, STM32CubeMx, gcc v9.2.1     | 0.249000 sec | 0.011000 sec | 31303 bytes totals    | -O2, Single precision
STM32F411RE, Arm Cortex-M4 core, 100 MHz, 128 KB SRAM, **HW FPU**, No OS, STM32CubeMx, gcc v9.2.1     | 2.933000 sec | 0.119000 sec | 37843 bytes totals    | -O2, **Double** precision
STM32F411RE, Arm Cortex-M4 core, 100 MHz, 128 KB SRAM, **No FPU**, No OS, STM32CubeMx, gcc v9.2.1     | 1.798000 sec | 0.076000 sec | 33743 bytes totals    | -O2, Single precision

Note: 
- I ported the [lm_ellipsoid_fit.c](./examples/lm_ellipsoid_fit/lm_ellipsoid_fit.c) to the Cortex-M4 mentioned above, the CPU takes about 0.012000 seconds to complete computation.
    - -O2, Single precision, HW FPU
    - From compute X' \* X, finding the eigenvalue and eigenvectors of given square matrix to resolving center point offset of sphere,
    - The printf/dump functions are disabled during the test.


#### (2) Result (basic loop version GEMM and AXPY):

Platform                                                                                              |     GEMM     |     AXPY     | Code size in lm_lib.a |    Note 
------------------------------------------------------------------------------------------------------|:------------:|-------------:| ---------------------:| ---------------------
PC, AMD Ryzen 9 5900HS, 3.30 GHz, 8 cores, 32 GB ram, VirtualBox & Linux 64, gcc v9.3.0               | 0.000922 sec | 0.000065 sec | ?                     | -O2, Single precision
Raspberry Pi 3, ARM Cortex-A53, ARMv7 (v7l), 1.2 GHz, 4 cores, 1 GB ram, Raspbian, gcc v6.3.0         | 0.043838 sec | 0.001529 sec | ?                     | -O2, Single precision
STM32F411RE, Arm Cortex-M4 core, 100 MHz, 128 KB SRAM, **HW FPU**, No OS, STM32CubeMx, gcc v9.2.1     | 0.406000 sec | 0.014000 sec | ?                     | -O2, Single precision
STM32F411RE, Arm Cortex-M4 core, 100 MHz, 128 KB SRAM, **No FPU**, No OS, STM32CubeMx, gcc v9.2.1     | 2.322000 sec | 0.080000 sec | ?                     | -O2, Single precision


# Others
### Test code coverage
```
(1) Enable the Test-coverage option in Makefile
(2) make clean; make
(3) Execute the lm_ut
(4) Execute the run_coverage.sh and check the HTML output (ut_coverage/index.html)
```

### Profile the functions
```
(1) Enable the Profiling option in Makefile
(2) make clean; make
(3) Execute the lm_ut
(4) Execute the run_gprof.sh and check the output
```

### Generate the Doxygen document
```
Most of public functions have been well commented in doxygen style format,
please use Doxygen tool to scan and generate the Doxygen documents directly.
```


# TODO
- Most of the functions in this library have not been completely optimized, I believe that there is still a lot of room for optimization, especially the AXPY and GEMM function which should be able to be optimized according to different CPU architecture and instruction set in future.
- I am not in a hurry to optimize these functions right now because the "performance requirements" are unclear to me so far.


# Reference 
(*Thanks to everyone who shared the knowledge with people*)
- Gene H. Golub, Charles F. Van Loan, Matrix Computation 4th edition.
- BLAS (Basic Linear Algebra Subprograms), http://www.netlib.org/blas/
- Grant Sanderson, Essence of linear algebra, https://youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab
- and others people that I didn't mention.
