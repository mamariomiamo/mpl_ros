## When running the ellipsoid_planner, following files are used:

### mpl_test_node/launch/ellipsoid_planner_node/test.launch
- launch file that sets system parameters and start the ROS node **ellipsoid_planner_node**

### mpl_test_node/src/ellipsoid_planner_node.cpp
- ROS node that publishes to rviz
- Read map from file (in the example **`mpl_test_node/maps/office/office.bag`** is used.) and publish to point cloud
    *   [**Source code**](https://github.com/sikang/mpl_ros/blob/master/mpl_test_node/src/ellipsoid_planner_node.cpp#L26-L32)

- initialize **EllipsoidPlanner** object with
    * `std::unique_ptr<MPL::EllipsoidPlanner> planner_`, a child class of **[`PlannerBase`](#motion_primitive_libraryincludempl_plannercommonplanner_baseh)** defined in [`planner_base.h`](https://github.com/sikang/motion_primitive_library/blob/547ddcda7cbf496fd3a76945da727926333d640e/include/mpl_planner/common/planner_base.h)
    * `setMap(...)` function defined in [`ellipsoid_planner.h`](doc.md#mpl_external_plannerincludempl_external_plannerellipsoid_plannerellipsoid_plannerh) that creates a new point cloud environment object[`env_cloud`](doc.md#mpl_external_plannerincludempl_external_plannerellipsoid_plannerenv_cloudh), which is a child class of [**`env_base`**](doc.md#motion_primitive_libraryincludempl_plannercommonenv_baseh)
    * set relevant parameters using inherited functions

- Publish location of start and goal point cloud coordinates for rviz visual
    * Special note:
        * `Waypoint3D start`, by changing the fields (`use_pos`,`use_vel`.`use_acc` etc.) decides the 5-bits control flag (`motion_primitive_library/include/mpl_basis/waypoint.h` **Control Flag `Control::Control control`**) for the waypoint.
        * When `get_succ` is called, control flag will determine the creation of `Primitive3D` (`motion_primitive_library/include/mpl_basis/primitive.h`) object.

- Read prior trajectory (ToDo: investigate usage of this part)

- [Set input control](https://github.com/sikang/mpl_ros/blob/master/mpl_test_node/src/ellipsoid_planner_node.cpp#L142-L157) 
    * **`vec_E<VecDf> U`** is used to store the control input discretization sets
    * size of U is equals to (2*num+1)^dim
    * `setU(U)` sets the array of constant control input in the environment object

- Start planning thread `planner_->plan(start, goal)`
    * Refer to **[`PlannerBase`](doc.md##motion_primitive_libraryincludempl_plannercommonplanner_baseh)**

- Publish expanded nodes, trajectory for rviz.

- Calculate and report maxium attitude with `max_attitude(.)`

### motion_primitive_library/include/mpl_planner/common/planner_base.h
- Planner base class
- Planning Thread `bool plan(const Coord &start, const Coord &goal)`
    * when verbose is on, do some printing about the start, goal and enviornment parameter information
    * Check if the start point is free (ToDo: verify if this is a dummy function since it seems that it always return **True**).
    * Create pointer `planner_ptr` that points to `GraphSearch` object as defined in [`graph_search.h`](doc.md#motion_primitive_libraryincludempl_plannercommongraph_searchh)
    * Reset the state space by creating a new `StateSpace` object defined in [`state_space.h`](doc.md#motion_primitive_libraryincludempl_plannercommonstate_spaceh)
    * Set goal node and clear `expanded_nodes_` and `expanded_edges_` in the environment class **`ENV_`**
    * Call `planner_ptr->Astar(.)` to start the search. More in [`graph_search.h`](doc.md#motion_primitive_libraryincludempl_plannercommongraph_searchh)

### mpl_external_planner/include/mpl_external_planner/ellipsoid_planner/ellipsoid_planner.h
- Motion primitive planner using point cloud
- This is a child class of **[`PlannerBase`](#motion_primitive_libraryincludempl_plannercommonplanner_baseh)** defined in [`planner_base.h`](https://github.com/sikang/motion_primitive_library/blob/547ddcda7cbf496fd3a76945da727926333d640e/include/mpl_planner/common/planner_base.h)
    * `setMap(...)` creates a new point cloud environment object[`env_cloud`](doc.md#mpl_external_plannerincludempl_external_plannerellipsoid_plannerenv_cloudh), which is a child class of [**`env_base`**](doc.md#motion_primitive_libraryincludempl_plannercommonenv_baseh)

### mpl_external_planner/include/mpl_external_planner/ellipsoid_planner/env_cloud.h
- Child class of env_base
- Constructor initiates a `EllipsoidUtil` object defined in [`ellipsoid_util.h`](doc.md#mpl_external_plannerincludempl_external_plannerellipsoid_plannerellipsoid_utilh)
- Methods:
    * `get_succ(...)`
        * For every control input in `U`:
            * 3D primitive is constructed from an initial state (p,v) and an input control (u)

### motion_primitive_library/include/mpl_planner/common/env_base.h

### mpl_external_planner/include/mpl_external_planner/ellipsoid_planner/ellipsoid_util.h

### motion_primitive_library/include/mpl_planner/common/graph_search.h
- Graph Search class that implements A* and Lifelong Planning A* algorithms
- A* function ([Source code](https://github.com/sikang/motion_primitive_library/blob/547ddcda7cbf496fd3a76945da727926333d640e/include/mpl_planner/common/graph_search.h))
    ```cpp
  /**
   * @brief Astar graph search
   *
   * @param start_coord start state
   * @param ENV object of `env_base' class
   * @param ss_ptr workspace input
   * @param traj output trajectory
   * @param max_expand max number of expanded states, default value is -1 which
   * means there is no limitation
   */
  decimal_t Astar(const Coord &start_coord,
                  const std::shared_ptr<env_base<Dim>> &ENV,
                  std::shared_ptr<StateSpace<Dim, Coord>> &ss_ptr,
                  Trajectory<Dim> &traj, int max_expand = -1) 
    ```
    * Initialize start node in first iteration.
    * Pop from priority queue `pq_` and expand/get successors with `ENV->get_succ(...)` in [`env_cloud`](doc.md#mpl_external_plannerincludempl_external_plannerellipsoid_plannerenv_cloudh)


### motion_primitive_library/include/mpl_planner/common/state_space.h
- State space class for graph search
- Define two structures:
    * **`State`**
        * Single Lattice of the graph, this is a structure for each node in the search graph.
            * consists of state coordinate, coordinate of successors, action cost of successors, coordinates of predecessors, action id of predecessors, action cost of prececessors
            * start-to-state g value, rhs value (LPA*), heuristic cost, flags for open and close set.
    * **`StateSpace`**
        * State space consists of all the open set states that are stored in priority queue, a hashmap that stores all the nodes


