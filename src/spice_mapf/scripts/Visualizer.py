import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Arrow
import numpy as np
import os
import cv2
from natsort import realsorted

import spice_msgs.msg as spice_msgs

from Agent import Agent
from Map import Map


class Visualizer:
    def __init__(self, map: Map, agents: list[Agent], workcell) -> None:
        self.map = map
        self.agents = agents
        self.workcell = workcell
        self.fig = None
    
    def init_plot(self):
        if not self.map.has_map:
            return False
        
        aspect = len(self.map.map[0]) / len(self.map.map)
        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

        return True

    def visualize(self):
        if self.fig is None:
            if not self.init_plot():
                return
        Colors = ['yellow', 'blue', 'orange', 'pink', 'magenta', 'black', 'brown', 'lime']
        my_map = np.flip(np.transpose(self.map.map),1)
        self.ax.clear()
        
        # create boundary patch
        x_min = -0.5
        y_min = -0.5
        x_max = len(my_map) - 0.5
        y_max = len(my_map[0]) - 0.5
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)

        self.ax.add_patch(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor='none', edgecolor='gray'))
        for i in range(len(my_map)):
            for j in range(len(my_map[0])):
                if my_map[i][j]:
                    self.ax.add_patch(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray'))
        for workcell in self.workcell.workcell_locations:
            id, pos = workcell
            self.ax.add_patch(
                Rectangle(
                    (pos[1] - 0.25, y_max - 0.5 - pos[0]-0.25),
                    0.5,
                    0.5,
                    facecolor='green',
                    edgecolor='black', alpha=0.5
                    )
                )
        # draw agents:            
        for i, agent in enumerate(self.agents):
            agent.color = Colors[i % len(Colors)]
            self.ax.add_patch(Rectangle((agent.current_goal[1] - 0.25, y_max - 0.5 - agent.current_goal[0]-0.25), 0.5, 0.5, facecolor=agent.color,
                                          edgecolor='black', alpha=0.5))
            if agent.target_goal is not None:
                self.ax.add_patch(Rectangle((agent.target_goal[1] - 0.25, y_max - 0.5 - agent.target_goal[0]-0.25), 0.5, 0.5, facecolor=agent.color,
                                          edgecolor='black', alpha=0.5))
            
            # arrow to next_loc from current_pos
            y,x = agent.current_pos
            y_next, x_next = agent.next_loc
            dx = x_next -x
            dy = y_next -y
            arrow = Arrow(x,y_max-y-0.5,dx,-dy,width=0.2, facecolor=agent.color)
            self.ax.add_patch(arrow)

            # arrow to path from next loc
            if len(agent.path) > 0:
                y,x = agent.next_loc
                y_next, x_next = agent.path[0]
                dx = x_next -x
                dy = y_next -y
                arrow = Arrow(x,y_max-y-0.5,dx,-dy,width=0.2, facecolor=agent.color)
                self.ax.add_patch(arrow)
            
            # arrows for path
            for j in range(len(agent.path)-1):
                y,x = agent.path[j]
                y_next, x_next = agent.path[j+1]
                dx = x_next -x
                dy = y_next -y
                arrow = Arrow(x,y_max-y-0.5,dx,-dy,width=0.2, facecolor=agent.color)
                self.ax.add_patch(arrow)

            agent_draw_x = agent.current_pos[1]
            agent_draw_y = y_max - agent.current_pos[0] - 0.5
            circle = Circle((agent_draw_x, agent_draw_y), 0.3, facecolor=agent.color,
                                    edgecolor='black')
            circle.original_face_color = agent.color
            self.ax.add_patch(circle)
            status_color = 'red' if len(agent.path) == 0 else 'green'
            status_circle = Circle((agent_draw_x+0.1, agent_draw_y + 0.1), 0.15, facecolor=status_color,
                                    edgecolor=status_color)
            self.ax.add_patch(status_circle)
            self.ax.text(agent_draw_x, agent_draw_y, agent.id.id)



        plt.pause(0.1)

    def save_fig(self):
        folder = 'plots'
        if not os.path.exists(folder):
            os.mkdir(folder)
        path = 'plots/'+str(self.time_interpolated)+'.png'
        self.fig.savefig(path)
    
    def make_video(self):
        image_folder = 'plots'
        images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
        images = realsorted(images)

        frame = cv2.imread(os.path.join(image_folder, images[0]))
        height, width, layers = frame.shape

        video = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (width,height))

        for image in images:
            video.write(cv2.imread(os.path.join(image_folder, image)))
        #cv2.destroyAllWindows()
        video.release()

    def show(self):
        plt.show()