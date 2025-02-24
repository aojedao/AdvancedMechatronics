#Rotate and Then Move 
#-----------------------------------------
#Interstellar BGM play
#Allright cooper the time is 9.00PM. We have 5 mini fireballs, 1 pack of Marlbro 100's and a vape
#Let the hacking begin

import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
import time

# Baisc Bitch
wheel_size = 0.021  # meters, 42mm Pololu wheel, smol but mighty
distance_between_wheels = 0.2  # meters, how far these bad boys are apart
encoder_ticks_per_spin = 48  # ticks per spin, counting every flex
time_per_move = 0.05  # seconds, updating on the reg, no slack
move_speed = 0.2  # m/s, zooming at a vibe pace
turn_speed = 1.0  # rad/s, spinning quick like TikTok trends
body_length = 0.3  # meters, thicc robot bod—slay queen
body_width = distance_between_wheels  # keeping it chill and simple
angle_wiggle_room = 0.05  # radians, close enough for this sus turner
distance_wiggle_room = 0.01  # meters, too close and I’m calling the cops, fam

#The Road not taken by Robert Frost
#Why did people stop writing poems, down fall of humanity
# Where we start and keep track of things,  IT all started with a big band
x_pos, y_pos, direction = 0.0, 0.0, 0.0  # starting point and facing
left_encoder_count, right_encoder_count = 0, 0  # counting wheel spins
left_extra, right_extra = 0.0, 0.0  # leftover bits of ticks
# the basic idea is to see how much ticks on the encoder is left. This is just for simulating a encoder with the brain i have
robot_path = [(x_pos, y_pos)]  # where we've been
goal_x, goal_y = None, None  # where we're headed

heading_to_goal = False #check if the robot is moving towards the goal
last_key = None  # what we pressed; could be a flag too

# Setting up the drawing canvas
plt.ion()  # makes it live-update, today's news in GAZA; HAHAHAHA

#Draw the plot PICASSO or maybe H!TL3R 
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_title('Robot Adventure Time (WASD or XY)')# title slaps, no cap
ax.grid(True)
robot_shape, = ax.plot([], [], 'b-', lw=2)  # I'm in love with the shape of you
left_wheel_dot, = ax.plot([], [], 'ro', ms=8)  # push and pull like a magnet do
right_wheel_dot, = ax.plot([], [], 'ro', ms=8)  # Although my heart is falling too
trail, = ax.plot([], [], 'g--', lw=1)  # Im in love with your body
goal_spot, = ax.plot([], [], 'kx', ms=10)  # And last night you were in my room
front_arrow, = ax.plot([], [], 'b-', lw=2)  # SHape of YOUUU
planned_route, = ax.plot([], [], 'r--', lw=1)  # Taylor Swift can never beat Micheal Jackson

def figure_out_ticks(left_speed, right_speed, time_step):
    global x_pos, y_pos, direction, left_encoder_count, right_encoder_count, left_extra, right_extra, robot_path
    #tried to add this variable in the main frame but shitz all crap bruh
    # How much each wheel turns
    left_turn = left_speed * time_step / wheel_size
    right_turn = right_speed * time_step / wheel_size
    
    # Add up ticks, keeping the leftovers
    left_extra += left_turn * encoder_ticks_per_spin / (2 * np.pi) #so not having 0.0 fucked my night up.
    #but the ticks have to be an int number, but i am dividing by pi
    #int / float = int stupid SOB
    # not I have the value in float; the number be HIGH for real for real
    right_extra += right_turn * encoder_ticks_per_spin / (2 * np.pi)
    #need to convert the float to int; so this might be a problem. 
    left_ticks = int(left_extra)
    right_ticks = int(right_extra)
    left_extra -= left_ticks
    right_extra -= right_ticks
    
    # Stay hydrated bois,
    left_encoder_count += left_ticks
    right_encoder_count += right_ticks
    
    # Figure out how we move overall; 
    #https://rossum.sourceforge.net/papers/DiffSteer/
    #the formulae have been directly thrifted from here with no sense of what is hapenning.Spelling is worng but I won't correct it
    #Coz I am MAFIA MAMA MIA
    forward_speed = (right_speed + left_speed) / 2
    spin_speed = (right_speed - left_speed) / distance_between_wheels
    x_pos += forward_speed * np.cos(direction) * time_step
    y_pos += forward_speed * np.sin(direction) * time_step
    direction += spin_speed * time_step
    direction = np.arctan2(np.sin(direction), np.cos(direction))  # keep it neat all fax no printer
    
    # Add this spot to our trail; CREEPY MoFo is tracking it's target. CALL THE COPS on this Bitch
    robot_path.append((x_pos, y_pos))
    return left_ticks, right_ticks

def draw_robot():
    # Draw the robot’s body
    #Feel the curves on this one.
    #Lost 14553 strands of hair figuring how to keep the wheels attached to the robot
    #why the fuck does everything have to be spoon fed to python Grow up SON, you on your own NOW.
    half_long = body_length / 2 #this just how rectangles work in python, stupid to the core. Do people who write these code have 2.5 braincells. Mom's must be smoking when they had them 
    half_wide = body_width / 2
    corners = [(-half_long, -half_wide), (half_long, -half_wide),
               (half_long, half_wide), (-half_long, half_wide), (-half_long, -half_wide)]#see complicated , I don't want to recreate the animations of AVATAR for an oscar. All i wanted was some clean rectangle.
    #on a seperate note: FIREBALL is GGGGGGGGGOOOOOOOOOOOOOOOOOOOOOOOOOOOOOODDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    rx, ry = [], []
    for cx, cy in corners:
        rx.append(x_pos + cx * np.cos(direction) - cy * np.sin(direction))
        ry.append(y_pos + cx * np.sin(direction) + cy * np.cos(direction))
    robot_shape.set_data(rx, ry)

    # Place the wheels
    #Stephen Hawkings gonna be proud of me from heaven or Epstien Island. 
    #Can't stand nor speak properly and still cheated on his wife.
    #Wonder how did that man get girls
    #Probably coz of the size ; SIZE matters IYKYK

    left_x = x_pos - (distance_between_wheels / 2) * np.sin(direction)
    left_y = y_pos + (distance_between_wheels / 2) * np.cos(direction)
    right_x = x_pos + (distance_between_wheels / 2) * np.sin(direction)
    right_y = y_pos - (distance_between_wheels / 2) * np.cos(direction)
    left_wheel_dot.set_data([left_x], [left_y]) #this is something I learnt new. set_data will save it as a fixed point so that It updates every time. also faster operation.
    right_wheel_dot.set_data([right_x], [right_y])

    # Show the front with an arrow
    #Ofcourse, when stalking someone we need to knwo tthe direction
    boner_size = half_long * 1.2  #hahahahah my robot is creepy
    #I need to stop drinking , someone please HELP. 
    #HEY The sun is up.
    #Also when does batman sleep, he's awake at night fighting crime and batwoman
    #In the day he runs a company
    #He worksout, but never gets enough sleep . How does that work. SLEEP == Recovery.
    #Is batman and alfred gay??? Most likely right, coz batman has dady and mommy issues and alfred was there when no one was around. Bees and the Bird talk must have been crazy.
    #*In intense Batman Voice* I'm BATMAN
    #BACK TO WORK
    arrow_x = [x_pos, x_pos + boner_size * np.cos(direction)]
    arrow_y = [y_pos, y_pos + boner_size * np.sin(direction)]
    front_arrow.set_data(arrow_x, arrow_y)

    # Draw where we’ve been
    #GOD I HATE MY MIND
    px, py = zip(*robot_path)
    trail.set_data(px, py)

    # Show the goal and planned route. MISSION GET THE GIRL.
    #We should name our robot TEB BUNDY.
    if goal_x is not None and goal_y is not None: 
        goal_spot.set_data([goal_x], [goal_y]) #the person who wrote set_data needs a pay raise. Remember to mail Guido Van Rossum about this.
        #Also why has no one written a simulator in python for 2D mobile robot forward kinematics.
        #It's always gazebo and ROS and fancy shit.
        #Start with the basics
        #HELLO WORLD
        planned_route.set_data([x_pos, goal_x], [y_pos, goal_y])
    else:
        planned_route.set_data([], [])

    # Keep everything in view
    #Never get caught (hashtag never get caught -> motto of every serial killer)
    all_x = px + (goal_x,) if goal_x else px
    all_y = py + (goal_y,) if goal_y else py
    ax.set_xlim(min(all_x) - 0.5, max(all_x) + 0.5)
    ax.set_ylim(min(all_y) - 0.5, max(all_y) + 0.5)
    plt.draw()
    plt.pause(0.001)

#We need to update the correct estimate of the robot at every postion and update the error, so that we can keep moving forward

#Why is that comment so clean , I need more alcohol
def tell_me_where_we_are(left_ticks, right_ticks):
    print(f"Left Encoder: {left_encoder_count} (+{left_ticks}) | "
          f"Right Encoder: {right_encoder_count} (+{right_ticks})")
    print(f"At: x = {x_pos:.3f} m, y = {y_pos:.3f} m | Facing: {direction:.3f} rad")

#q to quit
#wasd in tribute of CSGO
#I thought of having moving straight as the default and when a or d was pressed the robot stops moving 
#Def bad idea, I have killed a bunch of poeple and ran over too many traffic ligts

def catch_key_press(event):
    global last_key, heading_to_goal
    if not heading_to_goal:  # only move with keys if we’re not targeting
        key = event.keysym.lower()
        if key in ['w', 'a', 's', 'd', 'q']:
            last_key = key
#God this code if the version of Fify shades of grey.
#Prof.K gonna kill me when he reads the code
def pick_a_spot():
    global goal_x, goal_y, heading_to_goal
    try:
        goal_x = float(x_input.get())
        goal_y = float(y_input.get())
        heading_to_goal = True
        status_label.config(text=f"Aiming for: ({goal_x:.2f}, {goal_y:.2f})")
        # Let’s tell folks what we’re planning
        #FBI on my watch

        print(f"Heading from ({x_pos:.3f}, {y_pos:.3f}) to ({goal_x:.3f}, {goal_y:.3f})")
        how_far = np.sqrt((goal_x - x_pos)**2 + (goal_y - y_pos)**2)
        which_way = np.arctan2(goal_y - y_pos, goal_x - x_pos)
        print(f"  That’s {how_far:.3f} meters away")
        print(f"  Need to face {which_way:.3f} radians")
    except ValueError:
        status_label.config(text="Oops, bad X or Y number")
#DID YOU KNOW: Google will not give you the lyrics for nothing like us; coz that song has too many obsenities.

def keep_moving():
    global last_key, heading_to_goal, goal_x, goal_y
    print("Tap WASD to slide around, or drop X,Y and hit ‘Go’ (Q to yeet outta here):")
    print("W: Zoom forward, A: Swerve left, S: Reverse it, D: Dip right")
    print("Starting fresh at x=0, y=0, facing 0, let’s gooo\n")

    update_count = 0
    redraw_every = 5  # don’t redraw every single time, it’s too much, think bro
    #"Sliding out for a quick CIG sesh, BRB fam."
    #Well Well Well, look who we have here. More Fireball. 
    #Do you know why we have 70% water on earth. It's just coz humans get dehydrated when they consume alcohol and we need water  to stay hydrated
    #Stay hydrated my boissssss

    while True: #Who am I to decide if anything is true or not.
        #Well this LOOP sucks, i have been tryig to fight for my life here for the past 20 mins.
        #Time now 6.14 AM.
        #Some more mind musings.
        #I mean there are two sides to a story, who are we to decide what is right and what is wrong. People act on the pre-defined notion that their actions lead 
        #to the betterment of their life. Even in anger  the instantaneous actions we take might feel like doing right by us without thinking about the consequences.
        #If my lawyer is better than my oponent's then is it a fair trail at the court of law. "Matter of Fact" is just a royal covering to say that present me with facts; which might be delusional; but get a notary to sign on it and it shall be considered a fact and a judgement will be given on that.
        #Democracy is a myth. 
        left_speed, right_speed = 0.0, 0.0
        #Also if santiago had longer hair will he look like JESUS??
        if heading_to_goal and goal_x is not None and goal_y is not None:
            how_far_x = goal_x - x_pos
            how_far_y = goal_y - y_pos
            distance_left = np.sqrt(how_far_x**2 + how_far_y**2) #sqrt(x2 + y2)
            angle_we_want = np.arctan2(how_far_y, how_far_x)

            angle_off = angle_we_want - direction
            if abs(angle_off) > np.pi: #hugh did not think for a sec that angles can be negative. What a looser!!!
                angle_off -= 2 * np.pi * np.sign(angle_off)

            if abs(angle_off) > angle_wiggle_room:
                spin = np.clip(turn_speed * angle_off / np.pi, -turn_speed, turn_speed)
                #NEW LEARNINGS ALERT: np.clip();
                #If your number tries to go below the minimum, it gets bumped up to that minimum. If it tries to go above the maximum, it gets pulled back to that maximum. Anything in between just stays as is.
                #np.clip(..., -turn_speed, turn_speed): This says, “Hey, don’t let the spin speed go wild. Keep it between −1.0 and 1.0 rad/s.” So if the calculation spits out something crazy like 2.5 or -3.7, it gets capped at 1.0 or -1.0 instead.
                #"np.clip is a slick NumPy move (yeah, that np vibe) that keeps your numbers from wildin’ out too high or crashing too low. It’s like putting bumpers on your ride—you can flex all you want, but you ain’t yeeting off the track."
                #Genz slang is basically victorian era english. And I'm Loving IT

                #Allright COPS I'aint goona lie. I wrote a seperate code block just to see what happens with no np.clip();
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
            #TIME NOW 8.43AM. I spent too much time on this function. No wonder I am not getting any internships
            #Need to visit my friends wedding; need to guuide him in loosing his V-card.
            #Bro needs me

            #ToKYO drift the shit oout of this mofoo
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
            break #literally, I'm headding for a cigar break!!! My vape is charged out

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
    #__main__ bitch outta your league too, ah
    #Side bitch out of your league too, ah
    #Look what you've done
    #I'm a motherfuckin' starboy. Can drink and code . That's a new skill
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

    #Should've stayed with her. Look where it got me
    #Smoking drinking vaping sometimes all at the same time
    #In NewYork, living alone ,  no life, in a shared apartment with a guitar, football and python.
    #On the other hand, my boss Santiago is married and he is stuck with python too
    #There is no one clear way in life is there???

    #Also alejandro looks like thomas the train engine.
    #Need to puke BRB
    
