Import("env")

# General options that are passed to the C++ compiler
env.Append(CXXFLAGS=["-Wno-volatile"])
#//!GN Wno-volatile is not valid for g++ >= 8.4 (don't know the exact version)

# General options that are passed to the C compiler (C only; not C++).
env.Append(CFLAGS=["-Wno-discarded-qualifiers", "-Wno-implicit-function-declaration", "-Wno-incompatible-pointer-types"])
