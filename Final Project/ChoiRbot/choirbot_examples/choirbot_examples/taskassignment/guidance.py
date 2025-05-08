import rclpy
from rclpy.executors import MultiThreadedExecutor
from choirbot.guidance.task import TaskGuidance, PositionTaskExecutor
from choirbot.optimizer import TaskOptimizer
import sys

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Initialize components
        opt_settings = {'max_iterations': 20}
        executor = PositionTaskExecutor()
        optimizer = TaskOptimizer(settings=opt_settings)
        
        guidance = TaskGuidance(optimizer, executor, None, 'pubsub', 'odom')
        
        # Use MultiThreadedExecutor for better async handling
        executor = MultiThreadedExecutor()
        executor.add_node(guidance)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            guidance.destroy_node()
            
    except Exception as e:
        guidance.get_logger().error(f"Execution error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
