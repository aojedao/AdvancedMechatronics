import numpy as np
from disropt.algorithms import DistributedSimplex
from disropt.functions import Variable
from disropt.problems import LinearProblem
from disropt.agents import Agent
from threading import Event

from .optimizer import Optimizer


class TaskOptimizer(Optimizer):

    def __init__(self, resolution_strategy: str='simplex', cost_function: str='euclidean', settings: dict=None):
        super().__init__(settings)

        if resolution_strategy is not None and not isinstance(resolution_strategy, str): # FIXME: put a list of possible choices
            raise TypeError("resolution_strategy must be a string")

        if cost_function is not None and not isinstance(cost_function, str): # FIXME: idem (cannot be None)
            raise TypeError("cost_function must be a string")
        
        self.resolution_strategy = resolution_strategy
        self.cost_function = cost_function
        self.algorithm = None
        self.halt_optimization = False
        self.task_list = None
        self.n_tasks = None
        self.agent = None
        self._read_settings(**settings)
    
    def initialize(self, guidance: 'Guidance', halt_event: Event=None):
        super().initialize(guidance, halt_event)

        # create agent
        self.agent = Agent(in_neighbors=self.guidance.in_neighbors, out_neighbors=self.guidance.out_neighbors,
                      communicator=self.guidance.communicator)
    
    def _read_settings(self, stop_iterations: int=None, max_iterations: int=1000, **kwargs):
        self.stop_iterations = stop_iterations
        self.max_iterations = max_iterations
    
    def create_problem(self, task_list):
        # Initialize task_list storage
        if not hasattr(task_list, 'tasks'):
            raise ValueError("Invalid task_list: missing 'tasks' attribute")
        if not isinstance(task_list.tasks, list):
            raise ValueError("task_list.tasks must be a list")
        if not hasattr(self, 'task_list') or self.task_list is None:
            self.task_list = type('', (), {'tasks': []})()  # Create object with empty tasks list
        
        # Filter tasks for this agent
        self.task_list.tasks = [
            {'task': t, 'owner': t.id == self.guidance.agent_id}
            for t in task_list.tasks
        ]
        self.n_tasks = self.guidance.n_agents
        
        # Validate tasks
        if not self.task_list.tasks:
            raise ValueError("No tasks available for optimization")
        if any(t.id >= self.n_tasks for t in task_list.tasks):
            raise ValueError(f"Task ID {t.id} exceeds max allowed ({self.n_tasks-1})")
        
        # Prepare problem data
        task_positions = np.array([np.array(t['task'].coordinates) for t in self.task_list.tasks])
        task_indices = [t.id for t in task_list.tasks]
        starting_position = np.array(self.guidance.current_pose.position[:2])

        # Generate cost and constraints
        c = self._generate_cost(task_positions, starting_position)
        A, b = self._generate_constraints()

        # Create optimization problem
        x = Variable(len(self.task_list.tasks))
        obj = c @ x
        constr = A.transpose() @ x == b
        problem = LinearProblem(objective_function=obj, constraints=constr)
        self.agent.problem = problem
        
        # Set communication label
        self.guidance.communicator.current_label = int(task_list.label)

        # Initialize algorithm
        self.algorithm = DistributedSimplex(self.agent, stop_iterations=self.stop_iterations)

    def _generate_cost(self, task_positions, starting_position):
        cost_vector = np.empty((len(self.task_list.tasks), 1))
        for idx, row in enumerate(task_positions):
            cost_vector[idx, :] = np.linalg.norm(row - starting_position)
        return cost_vector

def _generate_constraints(self):
    N = self.guidance.n_agents
    M = len(self.task_list.tasks)
    A = np.zeros((N + M, M))
    b = np.ones((N + M, 1))

    # Agent constraints
    for task_idx, task_data in enumerate(self.task_list.tasks):
        if task_data['owner']:
            A[self.guidance.agent_id, task_idx] = 1

    # Task constraints
    for task_idx in range(M):
        A[N + task_idx, task_idx] = 1

    return A, b
        
    def optimize(self):
        self.algorithm.run(self.max_iterations, event=self._halt_event)

    def get_result(self):
        try:
            x_basic = self.algorithm.x_basic
            basis = self.algorithm.B
            nonzero_basic = np.nonzero(x_basic)[0]
            
            assigned_tasks = []
            for idx in nonzero_basic:
                col = basis[1:, idx]
                if col[self.guidance.agent_id] > 0.9:  # Threshold for assignment
                    task_id = np.nonzero(col[self.n_tasks:])[0][0]
                    # Return the actual task object, not the wrapper
                    assigned_tasks.append(self.task_list.tasks[task_id]['task'])
                    
            return assigned_tasks
        except Exception as e:
            self.guidance.get_logger().error(f"Result error: {str(e)}")
            return []

    def get_cost(self):
        return self.algorithm.J
