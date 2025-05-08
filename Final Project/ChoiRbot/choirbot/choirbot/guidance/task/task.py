from rclpy.task import Future
from std_msgs.msg import Empty
from threading import Event

from choirbot_interfaces.srv import TaskCompletionService
from ...optimizer import TaskOptimizer
from .. import RobotData
from .executor import TaskExecutor
from ..guidance import OptimizationGuidance
from ..optimization_thread import OptimizationThread
from ...utils import OrEvent


class TaskGuidance(OptimizationGuidance):
    # classe per livello di guida per scenari robotici task-like
    # questa classe si occupa di eseguire i task che trova in coda
    # nel frattempo sta in ascolto per eventuali optimization trigger,
    # che vengono gestiti in base alla strategia dinamica/statica scelta

    def __init__(self, optimizer: TaskOptimizer, executor: TaskExecutor,
            data: RobotData, pose_handler: str=None, pose_topic: str=None):
        super().__init__(optimizer, TaskOptimizationThread, pose_handler, pose_topic)
        self.data = data
        self.task_executor = executor
        self.current_task = None
        self.task_queue = []
        self.completed_tasks = []

        # triggering mechanism to start optimization
        self.opt_trigger_subscription = self.create_subscription(
                Empty, '/optimization_trigger', self.start_optimization, 10)

        # task list and task completion services
        self.task_list_client = self.create_client(executor.service, '/task_list')
        self.task_completion_client = self.create_client(TaskCompletionService, '/task_completion')

        # guard condition to start a new task
        self.task_gc = self.create_guard_condition(self.start_new_task)

        # initialize task executor
        self.task_executor.initialize(self)

        # wait for services
        self.task_list_client.wait_for_service()
        self.task_completion_client.wait_for_service()

        self.get_logger().info('Guidance {} started'.format(self.agent_id))
    
    def start_optimization(self, _):
        self.get_logger().info('Optimization triggered: requesting task list')

        # remove all enqueued tasks
        self.task_queue = []

        # request updated task list
        request = self.task_executor.service.Request(agent_id=self.agent_id)
        future = self.task_list_client.call_async(request)

        # launch optimization
        self.optimization_thread.optimize(future)
    
    def _optimization_ended(self):
        result = self.optimizer.get_result()
        
        # Add validation
        valid_tasks = []
        for task in result:
            if not hasattr(task, 'seq_num'):
                self.get_logger().error(f"Invalid task object received: {task}")
                continue
            valid_tasks.append(task)
        
        self.task_queue = valid_tasks
        self.get_logger().info(f'Assigned valid tasks: {[t.seq_num for t in valid_tasks]}')
        self.task_gc.trigger()
    
    def start_new_task(self):
        # stop if there are no new tasks
        if not self.task_queue:
            return
        
        # get task
        task = self.task_queue.pop(0)
        
        if task.id != self.agent_id:
            self.get_logger().error(
                f"Task ownership mismatch! Task {task.seq_num} "
                f"belongs to agent {task.id} but assigned to {self.agent_id}"
            )
            return

        # check if task was already performed
        #added to check if we are starting a new task when  one is exeu
        if self.current_task is not None:
            self.get_logger().warning('Attempted to start new task while current task still running')
            return
        if task.seq_num in self.completed_tasks:
            # trigger new task and exit
            #added to debug
            self.get_logger().error(f"Agent {self.agent_id} received wrong task {task.seq_num} (owner: {task.id})")
            self.task_gc.trigger()
            return

        # execute task
        self.get_logger().info('Got new task (seq_num {}) - starting execution'.format(task.seq_num)) #to say that we have gotten a new task
        self.get_logger().info(
        'Agent{} new task assigned (seq_num {}): Position {}'.format(
            self.agent_id,
            task.seq_num,
            task.coordinates  # Assuming task has coordinates attribute
            )
            )#say that we have a new taks and the position for that task
        self.current_task = task
        self.task_executor.execute_async(self.current_task, self.task_ended)
    
    def task_ended(self):
        # log to console
        self.get_logger().info('Task completed (seq_num {})'.format(self.current_task.seq_num))
        self.get_logger().info('Agent{} has reached the position (Task seq_num {} completed)'.format(self.agent_id, self.current_task.seq_num)) #says that we have reached a position
        # add task to list of completed tasks
        self.completed_tasks.append(self.current_task.seq_num)

        # notify table that task execution has completed
        request = TaskCompletionService.Request()
        request.agent_id = self.agent_id
        request.task_seq_num = self.current_task.seq_num
        self.task_completion_client.call_async(request)

        # start a new task
        self.current_task = None
        self.task_gc.trigger()


class TaskOptimizationThread(OptimizationThread):

    data_ready_event = None
    data_ready_future = None
    
    def optimize(self, future: Future):
        self.data_ready_future = future

        # prepare handling of asynchronous request
        self.data_ready_event = Event()

        def unblock(_):
            self.data_ready_event.set()
        
        self.data_ready_future.add_done_callback(unblock)

        # call method of parent class
        super().optimize()

    def do_optimize(self):
        # wait for problem data to be ready (or for halt event)
        OrEvent(self.data_ready_event, self._halt_event).wait()
        
        # exit on halt
        if self._halt_event.is_set():
            return

        # initialize and start optimization
        self.guidance.get_logger().info('Data received: starting optimization')
        data = self.data_ready_future.result().tasks
        self.optimizer.create_problem(data)
        self.optimizer.optimize()
