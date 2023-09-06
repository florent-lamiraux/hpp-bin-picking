/// \mainpage hpp-bin-picking documentation
///
/// This package provides some tools to address bin-picking issues. It is
/// composed
/// of a library and a \link hpp::corbaServer::ServerPlugin plugin \endlink
/// implementing an \link hpp::corbaserver::bin_picking::BinPicking
/// idl interface \endlink.
///
/// The main classes of the library are
/// \li hpp::bin_picking::Effector that is aimed at testing the collision of
///     an end effector during a grasp without solving the grasp constraint.
///     The effector gathers the objects that are rigidly linked to a \link
///     hpp::pinocchio::gripper gripper \endlink
///     and tests the collision with selected objects. Note that the \link
///     hpp::manipulation::Handle handle \endlink defining the grasp should
///     not have any degree of freedom. A function is provided to discretize
///     handles with degrees of freedom: hpp::bin_picking::discretizeHandle
///
