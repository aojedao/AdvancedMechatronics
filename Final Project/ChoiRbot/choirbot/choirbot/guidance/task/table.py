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
        self.bipartite_graph = {} # keys correspond to seq_num, values are lists of agents allowed to perform that task
        self.largest_seq_num = 0
        self.label = 0

        self.get_logger().info('Task table started')
    
    def task_list_service(self, request, response):
        agent = request.agent_id
        filtered_tasks = [t for t in self.task_list_comm if agent in self.bipartite_graph[t.seq_num]]
        
        self.get_logger().info(
            f'Sending to agent {agent}:\n'
            f'Task IDs: {[t.seq_num for t in filtered_tasks]}\n'
            f'Positions: {[t.coordinates for t in filtered_tasks]}'
        )
        
        response.tasks = self.make_task_array(filtered_tasks)
        response.tasks.all_tasks_count = len(self.task_list_comm)
        response.tasks.label = self.label
        return response
    
    def task_completion_service(self, request, response):
        agent = request.agent_id
        task_seq_num = request.task_seq_num
        task = next(t for t in self.task_list if t.seq_num == task_seq_num)
        
        self.get_logger().info(
            f'Agent {agent} completed task {task_seq_num} '
            f'(Position: {task.coordinates}) '
            f'and reached target location'
        )
        
        # Mark task as complete
        index = self.task_list.index(task)
        del self.task_list[index]
        del self.bipartite_graph[task_seq_num]
        
        self.gc.trigger()
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
        """Generates tasks only for robots with empty queues, preserving all original structure"""
        # Store which robots need tasks (NEW)
        robots_needing_tasks = list(set(
            agent for agents in self.bipartite_graph.values() 
            for agent in agents
            if not any(task for task in self.task_list if agent in self.bipartite_graph[task.seq_num])
        )) or [0, 1]  # Default to all robots if empty

        # Clear only tasks for robots that need them (MODIFIED)
        self.task_list = [
            task for task in self.task_list
            if not any(agent in robots_needing_tasks for agent in self.bipartite_graph[task.seq_num])
        ]
        
        # Rebuild bipartite graph (MODIFIED)
        self.bipartite_graph = {
            task.seq_num: self.bipartite_graph[task.seq_num]
            for task in self.task_list
        }

        # Original hardcoded tasks (UNCHANGED)
        tasks_agent0 = [[0.16, 0.95], [3.0, 1.5]]  # ArUco 4
        tasks_agent1 = [[0.48, -0.06], [-3.0, 0.5]]  # ArUco 5

        # Only generate for robots needing tasks (MODIFIED)
        for robot_id in robots_needing_tasks:
            tasks = tasks_agent0 if robot_id == 0 else tasks_agent1
            for coords in tasks:
                task = PositionTask(
                    coordinates=coords,
                    id=self.gen_task_id(),
                    seq_num=self.gen_task_seq_num()
                )
                self.task_list.append(task)
                self.bipartite_graph[task.seq_num] = [robot_id]
                print(f'Assigned Task ID {task.id} (Seq {task.seq_num}) to ArUco ID {4 if robot_id == 0 else 5}: x={coords[0]}, y={coords[1]}')

        # Original housekeeping (UNCHANGED)
        self.task_list_comm = self.task_list.copy()
        self.times_tasks_generated += 1
        self.label += 1

    def can_generate_tasks(self):
        return len(self.task_list) < self.N #generate tasks when it is empty
