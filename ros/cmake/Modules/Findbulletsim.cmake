find_path( BULLETSIM_SQP_INCLUDE_DIR
  planning_problems2.h
  PATHS
    "$ENV{HOME}/bulletsim/"
    "$ENV{HOME}/research/bulletsim"
  PATH_SUFFIXES src/sqp
  )

# This is just because currently in bulletsim headers are alongside source files
get_filename_component(BULLETSIM_INCLUDE_DIR ${BULLETSIM_SQP_INCLUDE_DIR} PATH)

# For more ugly hacks (gets the root of the bulletsim dir)
get_filename_component(BULLETSIM_SOURCE_DIR ${BULLETSIM_INCLUDE_DIR} PATH)

find_library( BULLETSIM_SQP_LIB
  NAMES sqp2 libsqp2
  PATHS
    "$ENV{HOME}/bulletsim/"
    "$ENV{HOME}/research/bulletsim"
  PATH_SUFFIXES src/sqp
  )

find_library( BULLETSIM_SIMULATION_LIB
  NAMES simulation libsimulation
  PATHS
    "$ENV{HOME}/bulletsim/"
    "$ENV{HOME}/research/bulletsim"
  PATH_SUFFIXES src/simulation
  )

set(bulletsim_INCLUDE_DIRS ${BULLETSIM_INCLUDE_DIR})

set(bulletsim_LIBRARIES ${BULLETSIM_SQP_LIB} ${BULLETSIM_SIMULATION_LIB})

set(BULLET_DIR ${BULLETSIM_SOURCE_DIR}/lib/bullet-2.79)
set(BULLET_LIBS BulletFileLoader BulletSoftBody BulletDynamics BulletCollision LinearMath HACD)

# This is dumb
set(BULLET_SLIBS BulletSoftBody BulletDynamics BulletCollision LinearMath)

# TODO: Remove this terrible hack
# link_directories might make this less ugly but not sure, and it seems to be deprecated.
foreach(bullet_lib ${BULLET_SLIBS})
  set(BULLET_LOCAL_LIBS ${BULLET_LOCAL_LIBS} ${BULLET_DIR}/src/${bullet_lib}/lib${bullet_lib}.a)
endforeach(bullet_lib)

set(BULLET_LOCAL_LIBS ${BULLET_LOCAL_LIBS} ${BULLET_DIR}/Extras/HACD/libHACD.a)
set(BULLET_LOCAL_LIBS ${BULLET_LOCAL_LIBS} ${BULLET_DIR}/Extras/Serialize/BulletFileLoader/libBulletFileLoader.a)

message("HAX: ${BULLET_LOCAL_LIBS}")
include(FindPackageHandleStandardArgs)
set(CMAKE_FIND_LIBRARY_PREFIXES ${BULLETSIM_SOURCE_DIR}/lib ${CMAKE_FIND_LIBRARY_PREFIXES})
find_package_handle_standard_args(bulletsim DEFAULT_MSG
                                  BULLETSIM_SQP_LIB BULLETSIM_INCLUDE_DIR)