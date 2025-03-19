class NavigateToGoal(py_trees.behaviour.Behaviour, Node): # this class is a py_tree node and a ros node
    def __init__(self, name="NavigateToGoal"):
        super().__init__(name=name)
        py_trees.behaviour.Behaviour.__init__(self, name=name)
        self.motion_node = MotionNode() # create a motion node object

    def setup(self, **kwargs):
        """ Setup fcn to Hardware or driver initialisation, Middleware initialisation (e.g. ROS pubs/subs/services) or
        a parallel checking for a valid policy configuration after children have been added or removed"""
        self.node = kwargs['node']
        self.goal_publisher = self.node.create_publisher(PointStamped, '/motion/goal', 10)
        self.path_publisher = self.node.create_publisher(Path, '/motion/path', 10)

    def initialise(self):
        """ When is this called? The first time your behaviour is ticked and anytime the
        status is not RUNNING thereafter."""  
        self.timer = self.node.create_timer(60.0, self.timer_callback) # watchdog timer

    def update(self):
            """ Behavior Tree execution step. Called whenever the node is ticked """
            if self.motion_node.is_path:
                if self.motion_node.path_reached:
                    self.timer.cancel() 
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.RUNNING
            else:
                if self.motion_node.goal_reached_flag:
                    self.timer.cancel()
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.RUNNING

    def timer_callback(self):
        """ Callback function for the watchdog timer """
        return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status: py_trees.common.Status):
        """
        Minimal termination implementation.
        When is this called? Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        if new_status == py_trees.common.Status.SUCCESS:
            self.get_logger().info("[bhv] Goal reached successfully.")
        elif new_status == py_trees.common.Status.FAILURE:
            self.get_logger().info("[bhv] Failed to reach the goal.")