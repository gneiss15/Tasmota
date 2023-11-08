Import("env", "projenv")

import sys
stdout_fileno = sys.stdout

# Dump global construction environment (for debug purpose)
sys.stdout = open( '/home/gneiss/VSCode-linux-x64/data/Tasmota/build_output/build_env.log', 'w' )
print(env.Dump())
sys.stdout.close()

# Dump project construction environment (for debug purpose)
sys.stdout = open( '/home/gneiss/VSCode-linux-x64/data/Tasmota/build_output/build_projenv.log', 'w' )
print(projenv.Dump())
sys.stdout.close()

sys.stdout = stdout_fileno

# append extra flags to global build environment
# which later will be used to build:
# - frameworks
# - dependent libraries
#env.Append(CPPDEFINES=[
#  "MACRO_1_NAME",
#  ("MACRO_2_NAME", "MACRO_2_VALUE")
#])

# append extra flags to only project build environment
#projenv.Append(CPPDEFINES=[
#  "PROJECT_EXTRA_MACRO_1_NAME",
#  ("ROJECT_EXTRA_MACRO_2_NAME", "ROJECT_EXTRA_MACRO_2_VALUE")
#])
projenv.Append(CXXFLAGS=["-v"])