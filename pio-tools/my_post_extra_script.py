Import("env", "projenv")

# Dump global construction environment (for debug purpose)
#with open( '/home/gneiss/VSCode-linux-x64/data/Tasmota/build_output/build_env.log', 'w' ) as f:
#  print( env.Dump(), file = f )

# Dump project construction environment (for debug purpose)
#with open( '/home/gneiss/VSCode-linux-x64/data/Tasmota/build_output/build_projenv.log', 'w' ) as f:
#  print( projenv.Dump(), file = f )


# append extra flags to global build environment ( used to build: -frameworks - dependent libraries)
#env.Append(CPPDEFINES=["MACRO_1_NAME",...])

# append extra flags to only project build environment
#projenv.Append(CPPDEFINES=["PROJECT_EXTRA_MACRO_1_NAME",...])
#projenv.Append(CXXFLAGS=["-v"])

#def Set_Std_to_Cpp17(env, node):
#  """
#    `node.name` - a name of File System Node
#    `node.get_path()` - a relative path
#    `node.get_abspath()` - an absolute path
#  """
#  if not node.name.startswith( "xsns_114_sml." ):
#    return node
  
#  MyEnv = env.Clone()
  # replace 'std=gnu++11' of CXXFLAGS with 'std=c++17'

#  newFlags = []
#  for s in MyEnv[ 'CXXFLAGS' ]:
#    if s.startswith( "-std=" ):
#      newFlags.append( "-std=C++17" )
#    else:
#      newFlags.append( s )

#  MyEnv.Replace( CXXFLAGS = newFlags )

  # Dump special construction environment (for debug purpose)
#  with open( '/home/gneiss/VSCode-linux-x64/data/Tasmota/build_output/build_specialenv.log', 'w' ) as f:
#    print( MyEnv.Dump() , file = f )

#  return MyEnv

#projenv.AddBuildMiddleware( Set_Std_to_Cpp17, "xsns_114_sml.*" )

def ShowFiles( env, node ):
  with open( '/home/gneiss/VSCode-linux-x64/data/Tasmota/build_output/build_files.log', 'a' ) as f:
    print( node.name, file = f )

env.AddBuildMiddleware( ShowFiles )
