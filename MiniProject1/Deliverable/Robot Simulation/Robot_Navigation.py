#Rotate and Then Move - BLE Odometry simulation.
#-----------------------------------------


import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
import time

wheel_size = 0.021  # meters, 42mm Pololu wheel
distance_between_wheels = 0.2  # meters
encoder_ticks_per_spin = 48  # ticks per spin
time_per_move = 0.05  # seconds
move_speed = 0.2  # m/s
turn_speed = 1.0  # rad/s
body_length = 0.3  # meters
body_width = distance_between_wheels 
angle_wiggle_room = 0.05  # radians
distance_wiggle_room = 0.01  # meters

x_pos, y_pos, direction = 0.0, 0.0, 0.0  # starting point and facing
left_encoder_count, right_encoder_count = 0, 0  # counting wheel spins
left_extra, right_extra = 0.0, 0.0  # leftover bits of ticks
# the basic idea is to see how much ticks on the encoder is left. 
robot_path = [(x_pos, y_pos)]  # where we've been
goal_x, goal_y = None, None  # where we're headed

heading_to_goal = False #check if the robot is moving towards the goal
last_key = None  # what we pressed; could be a flag too

# Setting up the drawing canvas
plt.ion()  # makes it live-update


fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_title('Robot Adventure Time (WASD or XY)')# title slaps, no cap
ax.grid(True)
robot_shape, = ax.plot([], [], 'b-', lw=2) 
left_wheel_dot, = ax.plot([], [], 'ro', ms=8) 
right_wheel_dot, = ax.plot([], [], 'ro', ms=8) 
trail, = ax.plot([], [], 'g--', lw=1) 
goal_spot, = ax.plot([], [], 'kx', ms=10) 
front_arrow, = ax.plot([], [], 'b-', lw=2)
planned_route, = ax.plot([], [], 'r--', lw=1) 

def figure_out_ticks(left_speed, right_speed, time_step):
    global x_pos, y_pos, direction, left_encoder_count, right_encoder_count, left_extra, right_extra, robot_path
    #tried to add this variable in the main frame
    # How much each wheel turns
    left_turn = left_speed * time_step / wheel_size
    right_turn = right_speed * time_step / wheel_size
    
    # Add up ticks, keeping the leftovers
    left_extra += left_turn * encoder_ticks_per_spin / (2 * np.pi) 
    right_extra += right_turn * encoder_ticks_per_spin / (2 * np.pi)
    
    #need to convert the float to int; so this might be a problem. 
    left_ticks = int(left_extra)
    right_ticks = int(right_extra)
    left_extra -= left_ticks
    right_extra -= right_ticks
    
    left_encoder_count += left_ticks
    right_encoder_count += right_ticks
    
    # Figure out how we move overall; 
    #https://rossum.sourceforge.net/papers/DiffSteer/
    #the formulae have been directly thrifted from here with no sense of what is hapenning.Spelling is worng but I won't correct it
    forward_speed = (right_speed + left_speed) / 2
    spin_speed = (right_speed - left_speed) / distance_between_wheels
    x_pos += forward_speed * np.cos(direction) * time_step
    y_pos += forward_speed * np.sin(direction) * time_step
    direction += spin_speed * time_step
    direction = np.arctan2(np.sin(direction), np.cos(direction))  # keep it neat all fax no printer
    
    # Add this spot to our trail; CREEPY MoFo is tracking it's target.
    robot_path.append((x_pos, y_pos))
    return left_ticks, right_ticks

def draw_robot():
    # Draw the robot’s body
    half_long = body_length / 2
    half_wide = body_width / 2
    corners = [(-half_long, -half_wide), (half_long, -half_wide),
               (half_long, half_wide), (-half_long, half_wide), (-half_long, -half_wide)]
    rx, ry = [], []
    for cx, cy in corners:
        rx.append(x_pos + cx * np.cos(direction) - cy * np.sin(direction))
        ry.append(y_pos + cx * np.sin(direction) + cy * np.cos(direction))
    robot_shape.set_data(rx, ry)

    # Place the wheels
    left_x = x_pos - (distance_between_wheels / 2) * np.sin(direction)
    left_y = y_pos + (distance_between_wheels / 2) * np.cos(direction)
    right_x = x_pos + (distance_between_wheels / 2) * np.sin(direction)
    right_y = y_pos - (distance_between_wheels / 2) * np.cos(direction)
    left_wheel_dot.set_data([left_x], [left_y]) #this is something I learnt new. set_data will save it as a fixed point so that It updates every time. also faster operation.
    right_wheel_dot.set_data([right_x], [right_y])

    # Show the front with an arrow
    boner_size = half_long * 1.2 
    arrow_x = [x_pos, x_pos + boner_size * np.cos(direction)]
    arrow_y = [y_pos, y_pos + boner_size * np.sin(direction)]
    front_arrow.set_data(arrow_x, arrow_y)

    # Draw where we’ve been
    px, py = zip(*robot_path)
    trail.set_data(px, py)

    # Show the goal and planned route. 
    if goal_x is not None and goal_y is not None: 
        goal_spot.set_data([goal_x], [goal_y])
        planned_route.set_data([x_pos, goal_x], [y_pos, goal_y])
    else:
        planned_route.set_data([], [])

    all_x = px + (goal_x,) if goal_x else px
    all_y = py + (goal_y,) if goal_y else py
    ax.set_xlim(min(all_x) - 0.5, max(all_x) + 0.5)
    ax.set_ylim(min(all_y) - 0.5, max(all_y) + 0.5)
    plt.draw()
    plt.pause(0.001)

#We need to update the correct estimate of the robot at every postion and update the error, so that we can keep moving forward
def tell_me_where_we_are(left_ticks, right_ticks):
    print(f"Left Encoder: {left_encoder_count} (+{left_ticks}) | "
          f"Right Encoder: {right_encoder_count} (+{right_ticks})")
    print(f"At: x = {x_pos:.3f} m, y = {y_pos:.3f} m | Facing: {direction:.3f} rad")

def catch_key_press(event):
    global last_key, heading_to_goal
    if not heading_to_goal:  # only move with keys if we’re not targeting
        key = event.keysym.lower()
        if key in ['w', 'a', 's', 'd', 'q']:
            last_key = key

def pick_a_spot():
    global goal_x, goal_y, heading_to_goal
    try:
        goal_x = float(x_input.get())
        goal_y = float(y_input.get())
        heading_to_goal = True
        status_label.config(text=f"Aiming for: ({goal_x:.2f}, {goal_y:.2f})")

        print(f"Heading from ({x_pos:.3f}, {y_pos:.3f}) to ({goal_x:.3f}, {goal_y:.3f})")
        how_far = np.sqrt((goal_x - x_pos)**2 + (goal_y - y_pos)**2)
        which_way = np.arctan2(goal_y - y_pos, goal_x - x_pos)
        print(f"  That’s {how_far:.3f} meters away")
        print(f"  Need to face {which_way:.3f} radians")
    except ValueError:
        status_label.config(text="Oops, bad X or Y number")

def keep_moving():
    global last_key, heading_to_goal, goal_x, goal_y
    print("Tap WASD to slide around, or drop X,Y and hit ‘Go’ (Q to yeet outta here):")
    print("W: Zoom forward, A: Swerve left, S: Reverse it, D: Dip right")
    print("Starting fresh at x=0, y=0, facing 0, let’s gooo\n")

    update_count = 0
    redraw_every = 5  # don’t redraw every single time, it’s too much.
    
    while True: 
        left_speed, right_speed = 0.0, 0.0

        if heading_to_goal and goal_x is not None and goal_y is not None:
            how_far_x = goal_x - x_pos
            how_far_y = goal_y - y_pos
            distance_left = np.sqrt(how_far_x**2 + how_far_y**2) #sqrt(x2 + y2)
            angle_we_want = np.arctan2(how_far_y, how_far_x)

            angle_off = angle_we_want - direction
            if abs(angle_off) > np.pi: 
                angle_off -= 2 * np.pi * np.sign(angle_off)

            if abs(angle_off) > angle_wiggle_room:
                spin = np.clip(turn_speed * angle_off / np.pi, -turn_speed, turn_speed)
                left_speed = -spin * distance_between_wheels / 2
                right_speed = spin * distance_between_wheels / 2
                status_label.config(text=f"Turning, off by {angle_off:.3f} rad")
            elif distance_left > distance_wiggle_room:
                left_speed = move_speed
                right_speed = move_speed
                status_label.config(text=f"Rolling, {distance_left:.3f} m to go")
            else:
                heading_to_goal = False
                goal_x, goal_y = None, None
                status_label.config(text="Made it!")

           
        elif last_key == 'w':
            left_speed = move_speed
            right_speed = move_speed
        elif last_key == 's':
            left_speed = -move_speed
            right_speed = -move_speed
        elif last_key == 'a':
            left_speed = -move_speed * 0.5
            right_speed = move_speed
        elif last_key == 'd':
            left_speed = move_speed
            right_speed = -move_speed * 0.5
        elif last_key == 'q':
            print("PEACE OUT!")
            root.destroy()
            break 

        if left_speed != 0 or right_speed != 0:
            left_ticks, right_ticks = figure_out_ticks(left_speed, right_speed, time_per_move)
            tell_me_where_we_are(left_ticks, right_ticks)

            update_count += 1
            if update_count % redraw_every == 0:
                draw_robot()

        last_key = None
        root.update()
        time.sleep(time_per_move)

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Robot Buddy")
    root.geometry("300x200")
    root.bind("<KeyPress>", catch_key_press)

    tk.Label(root, text="Where to? X:").pack()
    x_input = tk.Entry(root)
    x_input.pack()
    tk.Label(root, text="Y:").pack()
    y_input = tk.Entry(root)
    y_input.pack()
    tk.Button(root, text="Go!", command=pick_a_spot).pack(pady=5)
    status_label = tk.Label(root, text="WASD or type X,Y to move")
    status_label.pack()

    try:
        keep_moving()
    except KeyboardInterrupt:
        print("\nYou stopped me!")
    except Exception as e:
        print(f"Uh oh: {e}")

    plt.ioff()
    plt.show()
