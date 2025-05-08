from rclpy.node import Node
from choirbot_interfaces.msg import PositionTask, PositionTaskArray
from choirbot_interfaces.srv import PositionTaskService, TaskCompletionService
from std_msgs.msg import Empty
import numpy as np

np.random.seed(4)

class TaskTable(Node):

    def __init__(self, N, service_type):
        super().__init__('task_table')
        self.N = N
        self.trigger_publisher = self.create_publisher(Empty, '/optimization_trigger', 10)
        self.gc = self.create_guard_condition(self.send_new_tasks)
        self.task_list_srv = self.create_service(service_type, '/task_list', self.task_list_service)
        self.task_completion_srv = self.create_service(TaskCompletionService, '/task_completion', self.task_completion_service)
        self.task_list = []
        self.task_list_comm = [] # task list to be communicated to agents
        #we don't need bipartate graph
        #self.bipartite_graph = {} # keys correspond to seq_num, values are lists of agents allowed to perform that task
        self.largest_seq_num = 0
        self.label = 0

        self.get_logger().info('Task table started')
    
    def task_list_service(self, request, response):
        agent_id = request.agent_id
        
        # Directly access the agent's dedicated task list
        agent_tasks = self.agent_tasks[agent_id]
        
        self.get_logger().info(
            f'Sending to agent {agent_id}:\n'
            f'Task IDs: {[t.seq_num for t in agent_tasks]}\n'
            f'Positions: {[t.coordinates for t in agent_tasks]}\n'
            f'Total system tasks: {sum(len(tasks) for tasks in self.agent_tasks.values())}'
        )
        
        response.tasks = self.make_task_array(agent_tasks)
        response.tasks.all_tasks_count = len(agent_tasks)  # Now per-agent count
        response.tasks.label = self.label
        return response
    
    def task_completion_service(self, request, response):
        agent_id = request.agent_id
        task_seq_num = request.task_seq_num
        
        # 1. First validate the agent_id exists
        if agent_id not in self.agent_tasks:
            self.get_logger().error(f'Invalid agent ID {agent_id}')
            response.success = False
            return response
        
        # 2. Then proceed with the existing task completion logic
        try:
            task = next(t for t in self.agent_tasks[agent_id] 
                       if t.seq_num == task_seq_num)
            
            self.get_logger().info(
                f'Agent {agent_id} completed task {task_seq_num}\n'
                f'Position: {task.coordinates}\n'
                f'Remaining tasks for agent: {len(self.agent_tasks[agent_id])-1}'
            )
            
            # Remove from agent's task list
            self.agent_tasks[agent_id].remove(task)
            
            # Optional: Maintain a completed tasks log
            if not hasattr(self, 'completed_tasks'):
                self.completed_tasks = []
            self.completed_tasks.append(task)
            
            self.gc.trigger()
            response.success = True  # Explicit success flag
            return response
            
        except StopIteration:
            self.get_logger().error(
                f'Agent {agent_id} reported completion of unknown task {task_seq_num}'
            )
            response.success = False
            return response


    def gen_task_id(self):
        """
        Generate cyclic task IDs between 0 and N-1
        Returns:
            int: Task ID between 0 and N-1
        """
        if not self.task_list:  # Explicit empty list handling
            return 0
        try:
            return (max(t.id for t in self.task_list) + 1) % self.N
        except ValueError as e:
            self.get_logger().error(f"Task ID generation failed: {str(e)}")
            return 0  # Fallback value  
            
    def gen_task_seq_num(self):
        # sequence numbers are sequential (i.e. not recycled)
        seq_num = self.largest_seq_num
        self.largest_seq_num += 1
        return seq_num
    
    def send_new_tasks(self):
        if not self.can_generate_tasks():
            return # no need to generate new tasks
        
        self.get_logger().info('Generating new tasks and triggering optimization')
        self.generate_tasks()
        msg = Empty()
        self.trigger_publisher.publish(msg)
    
    def make_task_array(self, task_list):
        raise NotImplementedError
    
    def generate_tasks(self):
        raise NotImplementedError

    def can_generate_tasks(self):
        raise NotImplementedError

class PositionTaskTable(TaskTable):

    def __init__(self, N):
        super(PositionTaskTable, self).__init__(N, PositionTaskService)
        self.times_tasks_generated = 0
    
    def make_task_array(self, task_list):
        return PositionTaskArray(tasks=task_list)
    
    def generate_tasks(self):
        """Generates and assigns hardcoded tasks per robot with debug logging"""
        # Initialize separate task storage
        self.agent_tasks = {
            0: [],  # Tasks for Agent 0 (ArUco 4)
            1: []   # Tasks for Agent 1 (ArUco 5)
        }
        self.get_logger().info(f"Generating NEW tasks with label {self.label}")
        required_attrs = ['id', 'seq_num', 'coordinates']
        for agent_id, tasks in self.agent_tasks.items():
            for task in tasks:
                self.get_logger().info(
                    f"Agent {agent_id} Task {task.seq_num}: "
                    f"ID {task.id} at {task.coordinates}"
                )
        # Hardcoded positions 
        tasks_agent0 = [[0.16, 0.95], [3.0, 1.5]]  # ArUco 4
        tasks_agent1 = [[0.48, -0.06], [-3.0, 0.5]]  # ArUco 5

        # Debug: Log before generation
        self.get_logger().info(
            f"Generating new tasks. Current counts - "
            f"Agent0: {len(self.agent_tasks[0])}, "
            f"Agent1: {len(self.agent_tasks[1])}"
        )

        # Generate tasks for Agent 0
        for coords in tasks_agent0:
            task = PositionTask(
                coordinates=coords,
                id=0,
                seq_num=self.gen_task_seq_num()
            )
            self.agent_tasks[0].append(task)
            self.get_logger().debug(
                f"Created Task for Agent 0: "
                f"Seq {task.seq_num} at {coords}"
            )

        # Generate tasks for Agent 1  
        for coords in tasks_agent1:
            task = PositionTask(
            coordinates=coords,
            id=1,
            seq_num=self.gen_task_seq_num()
            )
            self.agent_tasks[1].append(task)
            self.get_logger().debug(
                f"Created Task for Agent 1: "
                f"Seq {task.seq_num} at {coords}"
            )

        # Debug: Log final assignments
        self.get_logger().info(
            f"Task generation complete. Final counts - "
            f"Agent0: {len(self.agent_tasks[0])} tasks, "
            f"Agent1: {len(self.agent_tasks[1])} tasks"
        )
        
        # Maintain compatibility with existing code
        self.task_list = self.agent_tasks[0] + self.agent_tasks[1]
        self.task_list_comm = self.task_list.copy()
        self.label += 1

    def can_generate_tasks(self):
        return len(self.task_list) < self.N #generate tasks when it is empty
