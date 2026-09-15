# Loaded by find_package(osqp_eigen_vendor): resolves the OsqpEigen config
# installed by the external project under this package's prefix and makes
# OsqpEigen::OsqpEigen available to the caller.
get_filename_component(
    _osqp_eigen_vendor_prefix
    "${osqp_eigen_vendor_DIR}/../../.."
    ABSOLUTE
)
find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(
    OsqpEigen
    REQUIRED
    CONFIG
    PATHS "${_osqp_eigen_vendor_prefix}/lib/cmake/OsqpEigen"
    NO_DEFAULT_PATH
)
unset(_osqp_eigen_vendor_prefix)
